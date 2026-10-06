"""Run the real ROS writer methods against an in-memory FC transport (no ROS/FC)."""
import ast
import __future__
from concurrent.futures import Future
from dataclasses import replace
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

from air_ground_landing.action_execution import (
    ActionExecutor as ActionLifecycle, ActionCommand, ActionKind, ActionRequest,
    ActionState, GUIDED_ACTIONS, VehicleSnapshot,
)
from air_ground_landing.guided_execution import PilotSessionGate, RcAuthorizationGate, RcGateConfig
from air_ground_landing.mavlink_ekf import decode_report, report_health
from air_ground_landing.vertical_latch import VerticalFaultLatch
from air_ground_landing.vertical_safety import VerticalHealthSnapshot, VerticalHealthState
from air_ground_landing.landing_alignment import LandingAlignment
from test_action_peripherals import Mixin as ActionPeripherals, Defaults as PERIPHERAL_DEFAULTS

ROOT = Path(__file__).resolve().parents[1]
ROS = ROOT / 'ros2_ws/src/air_ground_landing_ros2'
sys.path.insert(0, str(ROS))
from air_ground_landing_ros2.vertical_guard import VerticalGuard, VERTICAL_DEFAULTS


class PositionTarget(NS):
    FRAME_LOCAL_NED=1
    IGNORE_PX=1; IGNORE_PY=2; IGNORE_PZ=4
    IGNORE_AFX=64; IGNORE_AFY=128; IGNORE_AFZ=256
    IGNORE_YAW=1024; IGNORE_YAW_RATE=2048
    def __init__(self):
        super().__init__(header=NS(stamp=None), velocity=NS(x=0,y=0,z=0))


def load_writer():
    path=ROS/'air_ground_landing_ros2/action_executor.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    tree.body=[item for item in tree.body if isinstance(item,ast.ClassDef)]
    namespace=dict(globals(), Node=object, SetMode=NS(Request=NS),
                   CommandLong=NS(Request=NS),String=NS,
                   ExtendedState=NS(LANDED_STATE_ON_GROUND=1))
    exec(compile(tree,str(path),'exec',flags=__future__.annotations.compiler_flag),namespace)
    return namespace['ActionExecutorNode']


Writer=load_writer()


def load_capture_qualifier():
    path=ROS/'air_ground_landing_ros2/landing_target_adapter.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    tree.body=[item for item in tree.body if isinstance(item,ast.FunctionDef)
               and item.name=='capture_evidence_metadata']
    namespace=dict(math=math)
    exec(compile(tree,str(path),'exec'),namespace)
    return namespace['capture_evidence_metadata']


capture_evidence_metadata=load_capture_qualifier()


class FakeNode(Writer):
    def __init__(self, path, mode='enforce'):
        self.params={}
        self._declare_parameters()
        self.params.update(vertical_guard_state_path=str(path),vertical_guard_mode=mode)
        self.now=10.0
        for key,value in self.params.items():
            if key.endswith('_s') or key.endswith('_pwm') or key.endswith('_channel'):
                setattr(self,key,value)
        self.environment='offline'
        self.guided_mode='GUIDED'; self.land_mode='LAND'; self.entry_modes={'LOITER','ALT_HOLD'}
        self.allow_mode=self.allow_setpoint=self.output_enabled=True
        self.allow_disarm=self.allow_landing_disarm=self.allow_emergency_stop=False
        self.require_rc=True; self.rc_flight_flow_enabled=True
        self.lifecycle=ActionLifecycle()
        self.pilot_session=PilotSessionGate()
        self.pilot_session.enabled=True
        self.rc_gate=RcAuthorizationGate(RcGateConfig())
        self.vehicle_state=NS(mode='GUIDED',connected=True,armed=True)
        self.state_received_s=self.now
        self.pose=self.velocity=None
        self.pose_received_s=self.velocity_received_s=None
        self.extended=NS(landed_state=2); self.extended_received_s=self.now
        self.estimator_healthy=True; self.estimator_received_s=self.now
        self.ekf_report_received_s=None; self.ekf_report_count=0
        self.home_set=True; self.home_received_s=self.now
        self.rc_channels=(1500,1500,1500,1500,1500,1900,1500,1900)
        self.rc_received_s=self.now
        self.candidate=None; self.candidate_received_s=None
        self.target_healthy=False; self.target_aligned=False; self.target_status_received_s=None
        self.target_status_reason='NO_TARGET'; self.target_loss_sequence=0; self.landing_alignment=None
        self.range_m=1.; self.range_received_s=None
        self.landing_target_stream_healthy=False; self.landing_target_output_enabled=False
        self.landing_target_status_received_s=None
        self.mode_future=None; self.mode_request_action_id=None; self.mode_request_s=-math.inf
        self.owned_mode_deadlines={}; self.auto_land_inhibited=False
        self.command_future=None; self.command_request_s=-math.inf
        self.auto_action_sequence=0; self.last_auto_follow_attempt_s=-math.inf
        self.landing_switch_state='HIGH'; self.landing_switch_pwm=1900
        self.writes=[]; self.previews=[]; self.events=[]; self.mode_requests=[]; self.mode_futures=[]
        self.setpoint_publisher=NS(publish=self.writes.append)
        self.preview_publisher=NS(publish=self.previews.append)
        self.status_publisher=NS(publish=lambda msg:self.events.append(json.loads(msg.data)))
        def mode_call(request):
            self.mode_requests.append(request.custom_mode)
            result=Future(); self.mode_futures.append(result)
            return result
        self.mode_client=NS(service_is_ready=lambda:True,call_async=mode_call)
        self.command_client=NS(service_is_ready=lambda:True,call_async=lambda request:Future())
        self._init_peripherals(False)
        self._init_vertical_guard()
        self.vertical_rc_channels=self.rc_channels
        self.vertical_vehicle_state=self.vehicle_state
        self.vertical_extended=self.extended
        self.vertical_source_times.update(rc=self.now,state=self.now,extended=self.now)
        self.vertical_health=VerticalHealthSnapshot(VerticalHealthState.HEALTHY,'TEST',self.now,1.)

    def declare_parameter(self,name,value): self.params[name]=value
    def get_parameter(self,name): return NS(value=self.params[name])
    def create_publisher(self,*args): return NS(publish=lambda message:None)
    def create_client(self,*args): return NS(service_is_ready=lambda:False)
    def create_subscription(self,*args): pass
    def _now_s(self): return self.now
    def get_clock(self):
        return NS(now=lambda:NS(nanoseconds=int((1000+self.now)*1e9),to_msg=lambda:NS(sec=1010,nanosec=0)))
    def get_logger(self): return NS(error=lambda value:None)
    def stamp(self,age=0):
        stamp=1000+self.now-age
        sec=int(stamp)
        return NS(stamp=NS(sec=sec,nanosec=round((stamp-sec)*1e9)))
    def snapshot(self,**changes):
        data=dict(now_s=self.now,telemetry_fresh=True,connected=True,armed=True,
            mode=self.vehicle_state.mode,authorized=True,ekf_healthy=True,home_set=True,
            landed=False,position_enu=(0,0,1),velocity_enu=(0,0,0),yaw_rad=0.,
            candidate_fresh=True,candidate_velocity_flu=(.1,.1),range_m=1.,range_fresh=True)
        data.update(changes)
        return VehicleSnapshot(**data)
    def start_follow(self):
        return self.lifecycle.start(ActionRequest('follow',ActionKind.FOLLOW,30,{}),self.snapshot())
    def trip(self):
        self.vertical_latch.trip('FC_RANGE_CONTRADICTION')
        self._vertical_inhibit(self.now,self.vertical_latch.reason)


class WriterIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.path=Path(self.directory.name)/'vertical_guard.json'
        self.node=FakeNode(self.path)
    def tearDown(self): self.directory.cleanup()

    def test_enforce_final_writer_brakes_briefly_then_emits_nothing(self):
        n=self.node; n.start_follow(); n.trip()
        malicious=ActionCommand(desired_mode='LOITER',velocity_enu=(.4,.4,.2),yaw_rate_rad_s=.5)
        n._execute(malicious,n.snapshot(),n.now)
        self.assertEqual(len(n.writes),1)
        self.assertEqual(tuple(vars(n.writes[-1].velocity).values()),(0.,0.,0.))
        self.assertEqual(n.writes[-1].yaw_rate,0.)
        self.assertEqual(n.mode_requests,[])
        n.now+=.31
        n._execute(malicious,n.snapshot(),n.now)
        self.assertEqual(len(n.writes),1)
        self.assertEqual(n._vertical_status_payload(n.now)['vertical_guard_action'],'await_pilot')
        self.assertFalse(n._vertical_status_payload(n.now)['vertical_safe_hover_guaranteed'])

    def test_fault_blocks_rc_ch8_tag_recovery_external_requests_and_orphan(self):
        n=self.node; n.start_follow(); n.trip(); sequence=n.auto_action_sequence
        n._drive_rc_flight_flow(n.snapshot(landing_requested=True),n.now)
        n._request(NS(data=json.dumps({'action_id':'new','action':'FOLLOW','timeout_s':30,'params':{}})))
        n._request_mode('GUIDED',n.now)
        n.lifecycle.cancel(None,n.now,'CH8_LOW')
        self.assertTrue(n._peripheral_pre_tick(n.now))
        self.assertEqual(n.mode_requests,[])
        self.assertEqual(n.auto_action_sequence,sequence)
        self.assertEqual(n.events[-1]['reason'],'VERTICAL_GUARD_INHIBITED')
        self.assertTrue(n.vertical_latch.latched)

    def test_persisted_latch_blocks_shadow_restart_without_zero_transition(self):
        n=self.node; n.start_follow(); n.trip()
        restarted=FakeNode(self.path,'shadow')
        restarted._execute(ActionCommand(desired_mode='LOITER',velocity_enu=(0,0,.2)),restarted.snapshot(),restarted.now)
        restarted._request_mode('LOITER',restarted.now)
        self.assertTrue(restarted.vertical_inhibited)
        self.assertEqual(restarted.writes,[])
        self.assertEqual(restarted.mode_requests,[])

    def test_pilot_mode_heartbeat_ends_transition_even_if_snapshot_was_authorized(self):
        n=self.node; n.start_follow(); n.trip()
        n._state(NS(mode='STABILIZE',connected=True,armed=True,header=n.stamp()))
        n._execute(ActionCommand(velocity_enu=(.2,.2,.2)),n.snapshot(),n.now)
        self.assertEqual(n.writes,[])
        self.assertEqual(n.mode_requests,[])
        self.assertTrue(n.vertical_pilot_mode_observed)

    def test_native_land_is_not_rolled_back_by_fault_or_visual_loss(self):
        n=self.node; n.start_follow()
        n._state(NS(mode='LAND',connected=True,armed=True,header=n.stamp()))
        n.trip(); n.target_healthy=False
        n._execute(ActionCommand(desired_mode='LOITER',velocity_enu=(0,0,0)),n.snapshot(),n.now)
        n._request_mode('GUIDED',n.now)
        self.assertEqual(n.writes,[]); self.assertEqual(n.mode_requests,[])
        self.assertEqual(n._vertical_status_payload(n.now)['vertical_guard_action'],'native_land_fcu_owned')

    def test_pending_mode_ack_cannot_clear_fault_or_restart_outputs(self):
        n=self.node; n.start_follow(); n._request_mode('LAND',n.now)
        future=n.mode_future
        self.assertEqual(n.mode_requests,['LAND'])
        # A request already dispatched can finish at FC after the inhibit.
        n.trip()
        late=Future(); late.set_result(NS(mode_sent=True))
        n._mode_result(late,'follow','LAND')
        n._state(NS(mode='LAND',connected=True,armed=True,header=n.stamp()))
        n.now+=1
        n._request_mode('LOITER',n.now)
        self.assertTrue(n.vertical_latch.latched)
        self.assertEqual(n.mode_requests,['LAND'])
        self.assertTrue(future.cancelled())

    def test_shadow_fault_is_visible_but_does_not_change_command(self):
        n=FakeNode(self.path,'shadow'); n.start_follow()
        n.vertical_monitor=NS(update=lambda sample:VerticalHealthSnapshot(VerticalHealthState.FAULT,'CONTRADICTION',n.now))
        n._vertical_update(n.now)
        n._execute(ActionCommand(velocity_enu=(.1,.1,-.05)),n.snapshot(),n.now)
        self.assertEqual(n.writes[-1].velocity.z,-.05)
        self.assertFalse(n.vertical_latch.latched)
        self.assertEqual(n._vertical_status_payload(n.now)['vertical_state'],'FAULT')

    def test_enforce_unknown_inhibits_session_and_requires_explicit_ground_reset(self):
        n=self.node; n.start_follow()
        n.vertical_monitor=NS(update=lambda sample:VerticalHealthSnapshot(VerticalHealthState.UNKNOWN,'RANGE_STALE',n.now))
        n._vertical_update(n.now)
        self.assertTrue(n.vertical_inhibited)
        self.assertFalse(n.vertical_latch.latched)
        n.vertical_health=VerticalHealthSnapshot(VerticalHealthState.HEALTHY,'RECOVERED',n.now)
        n._drive_rc_flight_flow(n.snapshot(),n.now)
        self.assertFalse(n.lifecycle.active)

    def test_unknown_actual_sent_z_is_null_then_latest_sent_and_expires(self):
        n=self.node; n.start_follow()
        self.assertIsNone(n._vertical_status_payload(n.now)['commanded_vertical_speed_mps'])
        n._execute(ActionCommand(velocity_enu=(0,0,-.05)),n.snapshot(),n.now)
        self.assertEqual(n._vertical_status_payload(n.now)['commanded_vertical_speed_mps'],-.05)
        n.now+=1
        self.assertIsNone(n._vertical_status_payload(n.now)['commanded_vertical_speed_mps'])

    def test_reset_checks_all_ground_conditions_and_stable_health(self):
        n=self.node; n.trip(); n.vertical_reset_ready_since_s=n.now-4
        reset=NS(data='{"reset":true}')
        n._vertical_reset(reset)
        self.assertTrue(n.vertical_inhibited)  # Airborne/armed.
        n.vehicle_state.armed=False; n.extended.landed_state=1
        n.rc_channels=tuple(1100 if i==5 else x for i,x in enumerate(n.rc_channels))
        n.vertical_rc_channels=n.rc_channels
        n.vertical_source_times['state']=n.now-3
        n._vertical_reset(reset)
        self.assertTrue(n.vertical_inhibited)  # Stale state.
        n.state_received_s=n.now; n.extended_received_s=n.now; n.rc_received_s=n.now
        n.vertical_source_times.update(state=n.now,extended=n.now,rc=n.now)
        n.vertical_reset_ready_since_s=n.now-.5
        n._vertical_reset(reset)
        self.assertTrue(n.vertical_inhibited)  # Healthy dwell is too short.
        n.vertical_reset_ready_since_s=n.now-4
        n._vertical_ground_healthy=lambda now:True
        n.vertical_ground_samples=[(n.now-4,.06,0),(n.now,.06,0)]
        n.vertical_ground_health_s=n.now
        n._vertical_reset(reset)
        self.assertFalse(n.vertical_inhibited)
        self.assertFalse(VerticalFaultLatch(self.path).latched)
        self.assertFalse(n.pilot_session.enabled)

    def test_duplicate_source_pose_does_not_refresh_and_old_values_do_not_replace(self):
        n=self.node
        first=NS(header=n.stamp(),pose=NS(position=NS(x=0,y=0,z=1),orientation=NS(x=0,y=0,z=0,w=1)))
        n._pose(first); received=n.pose_received_s
        n.now+=.1
        repeated=NS(header=first.header,pose=NS(position=NS(x=0,y=0,z=9)))
        n._pose(repeated)
        self.assertGreater(n.pose_received_s,received)  # Legacy shadow path is unchanged.
        self.assertEqual(n.pose.pose.position.z,9)
        self.assertEqual(n.vertical_pose.pose.position.z,1)
        self.assertAlmostEqual(n.vertical_source_times['pose'],received)
        n._pose(NS(header=n.stamp(age=1),pose=first.pose))
        self.assertIsNone(n.vertical_source_times['pose'])

    def test_range_raw_sensor_duplicates_rebase_and_bad_quality(self):
        n=self.node
        status=dict(source='200/88',id=0,orientation=25,healthy=True,age_s=.05,
                    range_m=1,sensor_time_ms=100,rebase_count=0,signal_quality=90)
        n._vertical_range_status(NS(data=json.dumps(status)))
        original=n.vertical_range_measurement_s
        n.now+=.1; n._vertical_range_status(NS(data=json.dumps(status)))
        self.assertEqual(n.vertical_range_measurement_s,original)
        self.assertEqual(n.vertical_range_status['quality'],.9)
        status.update(rebase_count=1,sensor_time_ms=1)
        n._vertical_range_status(NS(data=json.dumps(status)))
        self.assertIsNone(n.vertical_range_measurement_s)
        self.assertEqual(n.vertical_range_status,{})

    def test_pnp_only_new_accepted_capture_frames_count(self):
        n=self.node
        n._pose(NS(header=n.stamp(age=.1),pose=NS(position=NS(x=0,y=0,z=1),
                    orientation=NS(x=0,y=0,z=0,w=1))))
        sample=dict(accepted_this_poll=True,accepted_capture_time_s=n.now-.1,
                    accepted_frame_id=100,accepted_vertical_distance_m=1,accepted_target_num=0,
                    accepted_time_basis='libcamera_sensor_timestamp',accepted_capture_timing_valid=True,
                    accepted_body_frd_m=[0.,0.,1.],accepted_body_frame='BODY_FRD')
        n._vertical_accept_pnp(sample,n.now); self.assertTrue(n.vertical_pnp)
        first=n.vertical_pnp.copy()
        n._vertical_accept_pnp(sample,n.now+.01); self.assertEqual(n.vertical_pnp,first)
        sample.update(accepted_frame_id=101,accepted_capture_time_s=n.now)
        n._vertical_accept_pnp(sample,n.now); self.assertTrue(n.vertical_pnp)
        n._vertical_accept_pnp(dict(sample,accepted_this_poll=False),n.now)
        self.assertEqual(n.vertical_pnp['time'],sample['accepted_capture_time_s'])

    def test_healthy_shadow_output_and_restart_do_not_create_session_inhibit(self):
        n=FakeNode(self.path,'shadow'); n.start_follow()
        n._execute(ActionCommand(velocity_enu=(0,0,-.05)),n.snapshot(),n.now)
        n._request_mode('LAND',n.now)
        self.assertEqual(len(n.writes),1)
        self.assertEqual(n.mode_requests,['LAND'])
        self.assertFalse(n.vertical_latch.session_active)
        self.assertFalse(json.loads(self.path.read_text(encoding='utf-8'))['session_active'])
        restarted=FakeNode(self.path,'shadow'); restarted.start_follow()
        self.assertFalse(restarted.vertical_inhibited)
        restarted._execute(ActionCommand(velocity_enu=(0,0,-.05)),restarted.snapshot(),restarted.now)
        self.assertEqual(len(restarted.writes),1)

    def test_capture_xyz_compensation_uses_nearest_original_attitude_not_latest_pose(self):
        n=self.node
        angle=math.radians(8.)
        n._pose(NS(header=n.stamp(age=.04),pose=NS(position=NS(x=0,y=0,z=1),
            orientation=NS(x=0,y=math.sin(angle/2),z=0,w=math.cos(angle/2)))))
        n.now+=.2
        n._pose(NS(header=n.stamp(),pose=NS(position=NS(x=0,y=0,z=1),
            orientation=NS(x=0,y=0,z=0,w=1))))
        sample=dict(accepted_this_poll=True,accepted_capture_time_s=9.95,
            accepted_frame_id=123,accepted_target_num=0,accepted_body_frd_m=[.4,0.,1.],
            accepted_body_frame='BODY_FRD',accepted_time_basis='libcamera_sensor_timestamp',
            accepted_capture_timing_valid=True)
        n._vertical_accept_pnp(sample,n.now)
        self.assertAlmostEqual(n.vertical_pnp['height'],math.cos(angle)+.4*math.sin(angle))
        self.assertAlmostEqual(n.vertical_pnp['attitude_time'],9.96)
        self.assertNotAlmostEqual(n.vertical_pnp['height'],1.)

    def test_derived_analysis_time_or_absent_misaligned_xyz_pose_cannot_qualify_pnp(self):
        n=self.node
        sample=dict(accepted_this_poll=True,accepted_capture_time_s=n.now-.05,
            accepted_frame_id=123,accepted_target_num=0,accepted_body_frd_m=[0.,0.,1.],
            accepted_body_frame='BODY_FRD',accepted_time_basis='libcamera_sensor_timestamp',
            accepted_capture_timing_valid=True)
        n._vertical_accept_pnp(sample,n.now)
        self.assertEqual(n.vertical_pnp,{})
        self.assertEqual(n.vertical_pnp_reason,'CAPTURE_ATTITUDE_UNAVAILABLE')
        n._pose(NS(header=n.stamp(age=.3),pose=NS(position=NS(x=0,y=0,z=1),
            orientation=NS(x=0,y=0,z=0,w=1))))
        n._vertical_accept_pnp(sample,n.now)
        self.assertEqual(n.vertical_pnp,{})
        self.assertEqual(n.vertical_pnp_reason,'CAPTURE_ATTITUDE_NOT_ALIGNED')
        n._pose(NS(header=n.stamp(),pose=NS(position=NS(x=0,y=0,z=1),
            orientation=NS(x=0,y=0,z=0,w=1))))
        for changes in ({'accepted_time_basis':'derived_analysis_time'},
                        {'accepted_capture_timing_valid':False},
                        {'accepted_body_frd_m':None}, {'accepted_body_frame':'BODY_FLU'}):
            n._vertical_accept_pnp(dict(sample,**changes),n.now)
            self.assertEqual(n.vertical_pnp,{})

    def test_capture_metadata_requires_exact_frame_clock_and_independent_sensor_time(self):
        observation=NS(source_sequence=15,capture_time_s=9.95)
        data=dict(analysis_sequence=15,capture_analysis_sequence=15,capture_timing_valid=True,
            capture_timestamp_source='libcamera_sensor_timestamp',capture_clock_id='CLOCK_MONOTONIC',
            capture_sensor_clock_id='CLOCK_BOOTTIME',capture_time_semantics='RPI_FIRST_PIXEL_EXPOSURE_START',
            capture_monotonic_s=9.9,capture_sensor_timestamp_ns=9900000000)
        qualified=capture_evidence_metadata(data,observation,10.)
        self.assertTrue(qualified['accepted_capture_timing_valid'])
        self.assertEqual(qualified['accepted_capture_time_s'],9.9)
        for changes in ({'capture_analysis_sequence':14},{'capture_timing_valid':False},
                        {'capture_clock_id':'CLOCK_REALTIME'},{'capture_monotonic_s':10.1},
                        {'capture_monotonic_s':9.0},{'capture_sensor_timestamp_ns':None}):
            rejected=capture_evidence_metadata(dict(data,**changes),observation,10.)
            self.assertFalse(rejected['accepted_capture_timing_valid'])
            self.assertIsNone(rejected['accepted_capture_time_s'])
        repeated=capture_evidence_metadata(data,observation,10.,9900000000)
        self.assertFalse(repeated['accepted_capture_timing_valid'])
        derived=capture_evidence_metadata({},observation,10.)
        self.assertEqual(derived['accepted_time_basis'],'derived_analysis_time')
        self.assertEqual(derived['accepted_capture_time_s'],9.95)
        self.assertFalse(derived['accepted_capture_timing_valid'])

    def test_durable_session_sentinel_survives_fault_record_write_failure(self):
        n=self.node; n.start_follow()
        n._execute(ActionCommand(velocity_enu=(0,0,-.05)),n.snapshot(),n.now)
        self.assertTrue(n.vertical_latch.session_active)
        self.assertEqual(len(n.writes),1)
        n.vertical_latch._write=lambda:(_ for _ in ()).throw(OSError('disk lost'))
        n.trip()
        restarted=FakeNode(self.path,'shadow')
        self.assertTrue(restarted.vertical_latch.latched)
        self.assertEqual(restarted.vertical_latch.reason,'UNFINISHED_AUTOMATIC_SESSION_RESTART')
        restarted._execute(ActionCommand(velocity_enu=(0,0,.1)),restarted.snapshot(),restarted.now)
        self.assertEqual(restarted.writes,[])

    def test_first_output_is_refused_when_active_session_cannot_be_written(self):
        n=self.node; n.start_follow()
        n.vertical_latch._write=lambda:(_ for _ in ()).throw(OSError('read only'))
        n._execute(ActionCommand(velocity_enu=(0,0,-.05)),n.snapshot(),n.now)
        n._request_mode('LAND',n.now)
        n._execute(ActionCommand(),n.snapshot(),n.now)
        self.assertEqual(n.writes,[])
        self.assertEqual(n.mode_requests,[])
        self.assertTrue(n.vertical_inhibited)

    def test_async_range_uses_status_measurement_not_newer_range_value(self):
        n=self.node
        n.vertical_range_min_m=.02; n.vertical_range_max_m=10.
        n.range_m=1.1
        n.vertical_source_times['range']=n.now
        n._vertical_range_status(NS(data=json.dumps(dict(source='200/88',id=0,orientation=25,
            healthy=True,age_s=.02,range_m=1.,sensor_time_ms=100,rebase_count=0,
            measurement_monotonic_s=n.now-.04,stamp_monotonic_s=n.now-.02))))
        sample=n._vertical_sample(n.now)
        self.assertEqual(sample.range_m,1.)
        self.assertEqual(sample.range_time_s,n.now-.04)
        self.assertTrue(sample.range_healthy)

    def ground_observation(self,n):
        n._state(NS(mode='LOITER',connected=True,armed=False,header=n.stamp()))
        n._rc(NS(channels=(1500,1500,1500,1500,1500,1100,1500,1100),header=n.stamp()))
        n._extended(NS(landed_state=1,header=n.stamp()))
        n._pose(NS(header=n.stamp(),pose=NS(position=NS(x=0,y=0,z=0),orientation=NS(x=0,y=0,z=0,w=1))))
        n._velocity(NS(header=n.stamp(),twist=NS(linear=NS(x=0,y=0,z=0),angular=NS(z=0))))
        n._estimator(NS(header=n.stamp(),attitude_status_flag=True,velocity_horiz_status_flag=True,
            pos_horiz_abs_status_flag=True,pos_horiz_rel_status_flag=True,
            pos_vert_abs_status_flag=True,pos_vert_agl_status_flag=True,velocity_vert_status_flag=True,
            vel_ratio=.1,pos_vert_ratio=.2,hagl_ratio=.1))
        n._vertical_range_status(NS(data=json.dumps(dict(source='200/88',id=0,orientation=25,
            healthy=True,age_s=0.,range_m=.06,min_range_m=.02,max_range_m=8.,
            sensor_time_ms=round(n.now*1000),rebase_count=0,
            measurement_monotonic_s=n.now,stamp_monotonic_s=n.now))))

    def test_real_ground_reset_at_6cm_preserves_airborne_unknown(self):
        n=self.node; n.trip()
        for i in range(34):
            n.now=10+i*.1
            self.ground_observation(n)
            n._vertical_update(n.now)
        self.assertEqual(n.vertical_health.state,'UNKNOWN')  # Below airborne minimum .08m.
        self.assertEqual(n.vertical_ground_health_reason,'GROUND_SOURCES_STABLE')
        self.assertTrue(n.vertical_latch.latched)
        n._vertical_reset(NS(data='{"reset":true}'))
        self.assertFalse(n.vertical_inhibited)
        self.assertFalse(n.vertical_latch.latched)
        self.assertFalse(n.pilot_session.enabled)

    def test_ground_reset_cannot_reuse_stale_original_rc_or_single_healthy_frame(self):
        n=self.node; n.trip()
        for i in range(34):
            n.now=10+i*.1
            self.ground_observation(n)
            n._vertical_update(n.now)
        n.vertical_source_times['rc']=n.now-1
        n.rc_received_s=n.now  # A repeated legacy receipt cannot pass reset.
        n._vertical_reset(NS(data='{"reset":true}'))
        self.assertTrue(n.vertical_latch.latched)
        self.assertEqual(n._vertical_status_payload(n.now)['vertical_reset_gate_reason'],'RC_SIGNAL_MISSING_OR_STALE')
        n.now+=1; self.ground_observation(n); n._vertical_update(n.now)
        n._vertical_reset(NS(data='{"reset":true}'))
        self.assertTrue(n.vertical_latch.latched)

    def test_estimator_fallback_retains_vertical_flags_and_metrics(self):
        n=self.node
        n._estimator(NS(header=n.stamp(),attitude_status_flag=True,velocity_horiz_status_flag=True,
            pos_horiz_abs_status_flag=True,pos_horiz_rel_status_flag=False,
            pos_vert_abs_status_flag=False,pos_vert_agl_status_flag=False,velocity_vert_status_flag=False,
            vel_ratio=.5,pos_vert_ratio=2.3,hagl_ratio=float('nan')))
        self.assertTrue(n.estimator_healthy)
        self.assertFalse(n.vertical_status['vertical_position_valid'])
        self.assertFalse(n.vertical_status['vertical_velocity_valid'])
        self.assertEqual(n.vertical_status['vertical_position_ratio'],2.3)
        self.assertIsNone(n.vertical_status['height_above_ground_ratio'])

    def test_shadow_keeps_legacy_receipts_separate_from_missing_raw_stamps(self):
        n=FakeNode(self.path,'shadow')
        invalid=NS(stamp=NS(sec=0,nanosec=0))
        n._state(NS(mode='GUIDED',connected=True,armed=True,header=invalid))
        n._rc(NS(channels=n.rc_channels,header=invalid))
        n._extended(NS(landed_state=2,header=invalid))
        n._pose(NS(header=invalid,pose=NS(position=NS(x=0,y=0,z=1))))
        n._velocity(NS(header=invalid,twist=NS(linear=NS(x=0,y=0,z=0))))
        self.assertEqual(n.state_received_s,n.now)
        self.assertEqual(n.rc_received_s,n.now)
        self.assertEqual(n.extended_received_s,n.now)
        self.assertEqual(n.pose_received_s,n.now)
        self.assertEqual(n.velocity_received_s,n.now)
        self.assertIsNone(n.vertical_source_times['pose'])
        self.assertIsNone(n.vertical_source_times['rc'])

    def test_unknown_enforce_cannot_issue_orphan_mode_change(self):
        n=self.node
        n.vertical_health=VerticalHealthSnapshot(VerticalHealthState.UNKNOWN,'MISSING',n.now)
        n._orphan_rollback_due=True
        n._request_mode('LOITER',n.now)
        self.assertEqual(n.mode_requests,[])

    def test_real_raw_sources_to_monitor_fault_to_final_writer(self):
        n=self.node
        detected=None
        for i in range(75):
            n.now=10+i*.05
            elapsed=max(0.,n.now-11.2)
            height=1.+elapsed*.6
            z=1.-elapsed*.6
            n._state(NS(mode='GUIDED',connected=True,armed=True,header=n.stamp()))
            n._rc(NS(channels=(1500,1500,1500,1500,1500,1900,1500,1900),header=n.stamp()))
            n._pose(NS(header=n.stamp(),pose=NS(position=NS(x=0,y=0,z=z),orientation=NS(x=0,y=0,z=0,w=1))))
            n._velocity(NS(header=n.stamp(),twist=NS(linear=NS(x=0,y=0,z=-.6 if elapsed else 0),angular=NS(z=0))))
            n._estimator(NS(header=n.stamp(),attitude_status_flag=True,velocity_horiz_status_flag=True,
                pos_horiz_abs_status_flag=True,pos_horiz_rel_status_flag=True,
                pos_vert_abs_status_flag=True,pos_vert_agl_status_flag=True,velocity_vert_status_flag=True))
            n._vertical_range_status(NS(data=json.dumps(dict(source='200/88',id=0,orientation=25,
                healthy=True,age_s=0.,range_m=height,min_range_m=.02,max_range_m=8.,
                sensor_time_ms=round(n.now*1000),rebase_count=0,
                measurement_monotonic_s=n.now,stamp_monotonic_s=n.now))))
            n._vertical_update(n.now)
            if i==22:
                self.assertEqual(n.vertical_health.state,'HEALTHY')
                n.start_follow()
            if i>=22:
                n._execute(ActionCommand(velocity_enu=(.1,.1,-.05)),n.snapshot(),n.now)
            if n.vertical_latch.latched and detected is None:
                detected=n.now
        self.assertIsNotNone(detected)
        self.assertLess(detected,12.5)
        self.assertTrue(n.vertical_inhibited)
        self.assertFalse(n.lifecycle.active)
        self.assertEqual(n.writes[-1].velocity.z,0.)
        self.assertEqual(n.mode_requests,[])
        self.assertIsNone(n._vertical_sent_vz(n.now))
        self.assertTrue(n._vertical_status_payload(n.now)['vertical_last_valid_source_times'])

    def test_corrupt_or_unwritable_latch_fails_closed(self):
        self.path.write_text('broken',encoding='utf-8')
        latch=VerticalFaultLatch(self.path)
        self.assertTrue(latch.latched)
        self.assertIsNotNone(latch.storage_error)
        bad=VerticalFaultLatch(self.path/'cannot-create.json')
        self.assertTrue(bad.latched)


class EkfDecodeTests(unittest.TestCase):
    def report(self,flags,length=22,metrics=(.1,.2,.3,.4,.5)):
        raw=struct.pack('<5fH',*metrics,flags)
        raw=raw.ljust(24,b'\0')
        return dict(framing_status=1,system_id=1,component_id=1,message_id=193,
                    length=length,payload64=[int.from_bytes(raw[i:i+8],'little') for i in range(0,24,8)])
    def test_vertical_validity_is_separate_from_legacy_horizontal_health(self):
        args=self.report(1|2|8)
        result=decode_report(**args)
        self.assertTrue(report_health(**args))
        self.assertFalse(result.vertical_position_valid)
        self.assertFalse(result.vertical_velocity_valid)
        result=decode_report(**self.report(1|2|4|8|32))
        self.assertTrue(result.vertical_position_valid)
        self.assertTrue(result.vertical_velocity_valid)
        self.assertAlmostEqual(result.pos_vert_variance,.3,places=6)
    def test_mavlink2_trailing_zero_flags_byte_and_nonfinite_metric(self):
        args=self.report(1|2|4|8|32,length=21,metrics=(float('nan'),.2,.3,.4,.5))
        result=decode_report(**args)
        self.assertTrue(result.vertical_position_valid)
        self.assertIsNone(result.velocity_variance)
        self.assertIsNone(decode_report(**dict(args,length=20)))


if __name__=='__main__': unittest.main()
