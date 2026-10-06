import unittest

from air_ground_landing.mavlink_ekf import report_health


class EkfReportTests(unittest.TestCase):
    def report(self, flags, **changes):
        payload = bytearray(22)
        payload[20:22] = flags.to_bytes(2, 'little')
        words = [int.from_bytes(payload[i:i + 8].ljust(8, b'\0'), 'little')
                 for i in range(0, len(payload), 8)]
        fields = dict(framing_status=1, system_id=1, component_id=1,
                      message_id=193, length=22, payload64=words)
        fields.update(changes)
        return report_health(**fields)

    def test_real_flight_controller_relative_position_flags_are_healthy(self):
        self.assertTrue(self.report(367))

    def test_missing_position_or_fault_flags_fail_closed(self):
        self.assertFalse(self.report(1 | 2))
        self.assertFalse(self.report(367 | 1024))
        self.assertFalse(self.report(367 | 32768))

    def test_foreign_or_malformed_report_is_ignored(self):
        self.assertIsNone(self.report(367, system_id=200))
        self.assertIsNone(self.report(367, framing_status=2))
        self.assertIsNone(self.report(367, length=20))

    def test_mavlink2_trimmed_zero_flags_byte_is_accepted(self):
        # At length 21 only the low flags byte is on the wire; MAVLink 2 may
        # trim a zero-valued trailing extension byte.
        self.assertTrue(self.report(367, length=21))


if __name__ == '__main__':
    unittest.main()
