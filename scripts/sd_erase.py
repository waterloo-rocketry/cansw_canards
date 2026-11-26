#!/usr/bin/env python3
"""Erase every written block on an SD card (raw device, no filesystem needed).

Scans the device in 1 MiB chunks and overwrites with zeros only the chunks that
contain data. Chunks that are already blank (all 0x00 or all 0xFF) are skipped,
which is faster and saves write cycles.

Usage (needs root / Administrator):
    Linux:    sudo python3 sd_erase.py /dev/sdX
    macOS:    diskutil unmountDisk /dev/diskN && sudo python3 sd_erase.py /dev/rdiskN
    Windows:  python sd_erase.py \\\\.\\PhysicalDriveN     (admin shell, volume offline)

Options:
    --stop-after-blank MB   stop once MB consecutive blank megabytes are seen
                            (good for sequential raw logs: no need to scan the
                            whole card)
    --dry-run               report what would be erased, write nothing
    --yes                   skip the confirmation prompt

THIS DESTROYS DATA. Double-check the device path before running.
"""
import argparse
import os
import sys

CHUNK = 1024 * 1024  # 1 MiB, a multiple of the 512-byte sector size
ZEROS = bytes(CHUNK)
ONES = b"\xff" * CHUNK


def is_blank(buf: bytes) -> bool:
    n = len(buf)
    return buf == ZEROS[:n] or buf == ONES[:n]


def erase_written(path: str, stop_after_blank_mb: int = 0, dry_run: bool = False) -> int:
    """Zero every non-blank chunk on `path`. Returns the number of bytes erased."""
    erased = 0
    blank_run = 0
    offset = 0
    mode = "rb" if dry_run else "r+b"
    with open(path, mode, buffering=0) as dev:
        while True:
            try:
                buf = dev.read(CHUNK)
            except OSError:
                break  # some platforms raise instead of returning b"" at end of device
            if not buf:
                break
            if is_blank(buf):
                blank_run += len(buf)
                if stop_after_blank_mb and blank_run >= stop_after_blank_mb * CHUNK:
                    break
            else:
                blank_run = 0
                if not dry_run:
                    dev.seek(offset)
                    dev.write(ZEROS[: len(buf)])
                erased += len(buf)
            offset += len(buf)
            if (offset // CHUNK) % 64 == 0:
                print(f"\r  scanned {offset / 2**30:.2f} GiB, "
                      f"erased {erased / 2**20:.0f} MiB", end="", flush=True)
        if not dry_run:
            dev.flush()
            try:
                os.fsync(dev.fileno())
            except OSError:
                pass
    print(f"\r  scanned {offset / 2**30:.2f} GiB, erased {erased / 2**20:.0f} MiB")
    return erased


def main() -> int:
    p = argparse.ArgumentParser(description="Erase all written blocks on an SD card.")
    p.add_argument("device", help="raw device path, e.g. /dev/sdX, /dev/rdiskN, \\\\.\\PhysicalDriveN")
    p.add_argument("--stop-after-blank", type=int, default=0, metavar="MB",
                   help="stop after this many consecutive blank MiB (default: scan whole card)")
    p.add_argument("--dry-run", action="store_true", help="scan only, write nothing")
    p.add_argument("--yes", action="store_true", help="skip confirmation")
    args = p.parse_args()

    if not args.dry_run and not args.yes:
        print(f"About to ERASE all data on {args.device}. This cannot be undone.")
        if input("Type the device path again to confirm: ").strip() != args.device:
            print("Aborted.")
            return 1

    try:
        erased = erase_written(args.device, args.stop_after_blank, args.dry_run)
    except PermissionError:
        print("Permission denied: run as root/Administrator and unmount the card first.")
        return 1
    verb = "would erase" if args.dry_run else "erased"
    print(f"Done: {verb} {erased / 2**20:.0f} MiB.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
