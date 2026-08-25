from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from flight_log_export import (
    CHUNK,
    build_quick_compact,
    prepare_sparse,
    quick_block_indices,
    set_periodic_streams,
)


class FakeMav:
    def __init__(self) -> None:
        self.calls = []

    def request_data_stream_send(self, *args) -> None:
        self.calls.append(args)


class FakeLink:
    def __init__(self) -> None:
        self.mav = FakeMav()


class FlightLogExportTests(unittest.TestCase):
    def test_staged_plan_compact_and_stream_guard(self) -> None:
        size = 16_818_052
        selected = quick_block_indices(size)
        self.assertEqual(selected[:2], [0, 1])
        self.assertEqual(selected[4_999], 4_999)
        self.assertEqual(selected[5_000], 159_089)
        self.assertEqual(len(selected), 32_779)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sparse = root / "log.BIN"
            bitmap, bitmap_path = prepare_sparse(sparse, 1000)
            self.assertEqual(len(bitmap), 12)
            self.assertEqual(bitmap_path.stat().st_size, 12)
            payload = bytes(index % 251 for index in range(1000))
            sparse.write_bytes(payload)
            compact = build_quick_compact(sparse, root / "quick.BIN", 180, 250)
            self.assertEqual(compact["output_size_bytes"], 430)
            self.assertEqual((root / "quick.BIN").read_bytes(), payload[:180] + payload[-250:])

        link = FakeLink()
        set_periodic_streams(link, 1, 1, False, 0)
        set_periodic_streams(link, 1, 1, True, 10)
        self.assertEqual(len(link.mav.calls), 6)
        self.assertTrue(all(call[3:] == (0, 0) for call in link.mav.calls[:3]))
        self.assertTrue(all(call[3:] == (10, 1) for call in link.mav.calls[3:]))
        self.assertEqual(CHUNK, 90)


if __name__ == "__main__":
    unittest.main()
