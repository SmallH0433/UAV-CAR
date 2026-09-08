import unittest
from pathlib import Path
import json
from air_ground_landing.landing_target_bridge import BridgeConfig, LandingTargetBridge

class CameraOwnershipTests(unittest.TestCase):
    def validate(self, **fields):
        config=json.loads((Path(__file__).resolve().parents[1]/'config/moving_landing.prototype.json').read_text())
        bridge=LandingTargetBridge(BridgeConfig.from_mapping(config))
        status=dict(sensor='ov9281',mode='apriltag',tag_family='tag36h11',analysis_size=[1280,800])
        status.update(fields)
        return bridge._validate_metadata(status)
    def test_new_telemetry_online_does_not_imply_camera_ownership(self):
        self.assertIsNone(self.validate(camera_owns_mavlink=False,flight_controller_connected=True))
    def test_legacy_false_allowed(self):
        self.assertIsNone(self.validate(flight_controller_connected=False))
    def test_legacy_true_rejected(self):
        self.assertEqual(self.validate(flight_controller_connected=True),'CAMERA_SERVICE_MUST_NOT_OWN_MAVLINK')
    def test_missing_rejected(self):
        self.assertEqual(self.validate(),'CAMERA_SERVICE_MUST_NOT_OWN_MAVLINK')
    def test_explicit_ownership_overrides_legacy_false(self):
        for value in (True,None,0,'false'):
            self.assertEqual(self.validate(camera_owns_mavlink=value,flight_controller_connected=False),
                             'CAMERA_SERVICE_MUST_NOT_OWN_MAVLINK')

if __name__=='__main__':
    unittest.main()
