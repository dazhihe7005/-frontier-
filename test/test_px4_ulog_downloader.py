#!/usr/bin/env python3

import importlib.util
import pathlib
import types
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "px4_ulog_downloader.py"
SPEC = importlib.util.spec_from_file_location("px4_ulog_downloader", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MissingRangesTest(unittest.TestCase):
    def test_coalesces_and_bounds_missing_bytes(self):
        received = bytearray([1, 0, 0, 0, 0, 1, 0])
        self.assertEqual(
            MODULE.missing_ranges(received, len(received), 3),
            [(1, 3), (4, 1), (6, 1)])

    def test_complete_buffer_has_no_ranges(self):
        self.assertEqual(MODULE.missing_ranges(bytearray([1, 1]), 2, 90), [])


class FilenameTest(unittest.TestCase):
    class Stamp:
        def __init__(self, value):
            self.value = value

        def to_sec(self):
            return self.value

    class Entry:
        id = 192
        time_utc = None

    def test_uses_px4_utc_and_id(self):
        entry = self.Entry()
        entry.time_utc = self.Stamp(1791817935)
        self.assertEqual(
            MODULE.ulog_filename(entry),
            "px4_20261012T151215Z_id192.ulg")

    def test_falls_back_to_id_without_utc(self):
        entry = self.Entry()
        entry.time_utc = self.Stamp(0)
        self.assertEqual(MODULE.ulog_filename(entry), "px4_log_id192.ulg")


class StoreLogTest(unittest.TestCase):
    def test_writes_valid_file_and_checksum_atomically(self):
        import tempfile

        entry = types.SimpleNamespace(
            id=7,
            size=len(MODULE.ULOG_MAGIC) + 4,
            time_utc=types.SimpleNamespace(to_sec=lambda: 0))
        payload = MODULE.ULOG_MAGIC + b"test"
        with tempfile.TemporaryDirectory() as directory:
            owner = object.__new__(MODULE.Px4UlogDownloader)
            owner.output_directory = directory
            path, existed, digest = owner.store_log(entry, payload)
            self.assertFalse(existed)
            self.assertEqual(pathlib.Path(path).read_bytes(), payload)
            self.assertIn(digest, pathlib.Path(path + ".sha256").read_text())
            _path, existed, _digest = owner.store_log(entry, payload)
            self.assertTrue(existed)


if __name__ == "__main__":
    unittest.main()
