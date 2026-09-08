"""Run on the Pi vision venv against a staging directory; no camera or MAVLink."""
import copy
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import types

import cv2
import numpy as np

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'ov9281_debug'))
sys.path.insert(0, str(WORKSPACE / 'air_ground_open_source/08_air_ground_landing/src'))

# The test exercises VisionState._tag without opening a camera. Provide minimal
# import-only stand-ins when the Raspberry Pi camera packages are unavailable.
try:
    import picamera2  # noqa: F401
except ModuleNotFoundError:
    camera_module = types.ModuleType('picamera2')
    camera_module.Picamera2 = object
    encoder_module = types.ModuleType('picamera2.encoders')
    encoder_module.MJPEGEncoder = object
    output_module = types.ModuleType('picamera2.outputs')
    output_module.FileOutput = object
    sys.modules.update({
        'picamera2': camera_module,
        'picamera2.encoders': encoder_module,
        'picamera2.outputs': output_module,
    })
try:
    import pupil_apriltags  # noqa: F401
except ModuleNotFoundError:
    detector_module = types.ModuleType('pupil_apriltags')
    detector_module.Detector = object
    sys.modules['pupil_apriltags'] = detector_module

from ov9281_tag_orientation import body_tag_orientation, load_body_extrinsics
from ov9281_unified_service import VisionState
from ov9281_dual_tag import parse_tag_specs, parse_tag_quality_specs
from air_ground_landing.landing_target_bridge import BridgeConfig, LandingTargetBridge
from air_ground_landing.mavros_frames import body_frd_pose_to_ros_baselink

config_path = Path(os.environ.get(
    'AIR_GROUND_LANDING_CONFIG',
    WORKSPACE / 'air_ground_open_source/08_air_ground_landing/config/moving_landing.prototype.json',
))
config = json.loads(config_path.read_text())
rbc, offset = load_body_extrinsics(config_path)
K = np.array([[800.,0,640.],[0,800.,400.],[0,0,1.]])
D = np.array([-.04,.01,0.,0.]).reshape(4,1)
pad_to_tag = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
fixtures = []
for heading in (0, 35, 90, 180, -90):
    for tilt in (0, 18):
        rz = cv2.Rodrigues(np.array([0.,0.,math.radians(heading)]))[0]
        rx = cv2.Rodrigues(np.array([math.radians(tilt),0.,0.]))[0]
        expected = rz @ rx
        state = VisionState.__new__(VisionState)
        state.camera_matrix,state.distortion=K,D
        state.body_extrinsics=(rbc,offset)
        state.tag_specs=parse_tag_specs('0:0.100:outer,1:0.020:inner')
        state.tag_quality_gates=parse_tag_quality_specs('0:20:3:10,1:20:3:10')
        state.range_corrections={None:dict(scale=1.,offset_m=0.,status='test')}
        state.args=SimpleNamespace(switch_to_inner_below_m=.35,tag_switch_hysteresis_m=.05,tag_selection_policy='outer_first')
        state.active_tag_id=None
        state.lock=threading.Lock()
        state.tag_object_points={}
        detections=[]
        for tag_id,spec in state.tag_specs.items():
            h=spec.size_m/2
            points=np.array([[-h,h,0.],[h,h,0.],[h,-h,0.],[-h,-h,0.]])
            state.tag_object_points[tag_id]=points
            undo=cv2.Rodrigues(np.array([0.,0.,math.pi/4 if tag_id==1 else 0.]))[0]
            rct=rbc.T @ expected @ pad_to_tag.T @ undo.T
            rvec=cv2.Rodrigues(rct)[0]
            corners=cv2.fisheye.projectPoints(points.reshape(1,4,3),rvec,np.array([.015,-.02,.45]),K,D)[0].reshape(4,2)
            detections.append(SimpleNamespace(tag_id=tag_id,corners=corners,decision_margin=60.,hamming=0))
        state.detector=SimpleNamespace(detect=lambda *args,**kwargs:detections)
        state._tag(np.zeros((800,1280),np.uint8))
        for obs in state.observations:
            o=obs['orientation']
            assert np.allclose(o['rotation_pad_to_body_frd'],expected,atol=2e-3),(heading,tilt,obs['tag_id'],o)
            assert o['valid']
            status=dict(obs,found=True,sensor='ov9281',mode='apriltag',analysis_size=[1280,800],pixel_source='Y_MONO',tag_family='tag36h11',frame_age_ms=10,analysis_sequence=1,flight_controller_connected=False,camera_owns_mavlink=False)
            result=LandingTargetBridge(BridgeConfig.from_mapping(config)).process_status(status,received_time_s=1.,wall_time_usec=1000000)
            assert result.accepted,result.reason
            assert result.packet is not None
            assert np.allclose(result.packet.q,o['quaternion_pad_to_body_frd_wxyz'])
            # MAVROS BODY raw callback right-multiplies q_roll_pi; roundtrip is -q.
            _,qros=body_frd_pose_to_ros_baselink((result.packet.x,result.packet.y,result.packet.z),result.packet.q)
            w,x,y,z=qros
            assert np.allclose([-x,w,z,-y],-np.array(result.packet.q))
            fixtures.append(status)
        assert np.allclose(state.observations[0]['orientation']['pad_forward_body_frd'],state.observations[1]['orientation']['pad_forward_body_frd'],atol=2e-3)
bad=copy.deepcopy(fixtures[-1]);bad['orientation']['rotation_camera_optical_to_body_frd']=[[-1,0,0],[0,-1,0],[0,0,1]]
assert LandingTargetBridge(BridgeConfig.from_mapping(config)).process_status(bad,received_time_s=1.,wall_time_usec=1000000).reason=='ORIENTATION_EXTRINSICS_MISMATCH'
bad=copy.deepcopy(fixtures[-1]);bad['orientation']['quaternion_pad_to_body_frd_wxyz']=[1.,0.,0.,0.]
assert LandingTargetBridge(BridgeConfig.from_mapping(config)).process_status(bad,received_time_s=1.,wall_time_usec=1000000).reason=='INVALID_BODY_ORIENTATION'
Path('body_orientation_test_fixtures.json').write_text(json.dumps(fixtures))
print('PASS: 20 distorted PnP detections -> common pad -> BODY_FRD -> LANDING_TARGET -> MAVROS quaternion roundtrip; mismatch rejection')
