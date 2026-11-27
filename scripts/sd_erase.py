#!/usr/bin/env python3
"""Erase every written block on an SD card (raw device, no filesystem needed).

Scans the device in 1 MiB pieces and overwrites with zeros every piece that is
not already all zeros. Afterwards the whole scanned area reads as 0x00, which
is what the logging firmware treats as blank. (Areas that read as 0xFF, which
some cards return after a hardware erase, are zeroed too, because the firmware
would treat them as already written.)

Usage (needs root / Administrator):
    Linux:    sudo python3 sd_erase.py /dev/sdX
    macOS:    diskutil unmountDisk /dev/diskN && sudo python3 sd_erase.py /dev/rdiskN
    Windows:  python sd_erase.py \\\\.\\PhysicalDriveN     (admin shell, no volume on the card)

Options:
    --stop-after-blank MB   stop once MB consecutive all-zero megabytes are seen
                            (only safe if all logged data is at the start of the
                            card)
    --dry-run               report what would be erased, write nothing
    --yes                   skip the confirmation prompt

Exit status:
    0  the erase (or dry run) covered the whole card, or stopped as requested
       by --stop-after-blank
    1  error: the card could not be opened, read, written or flushed, or the
       erase was aborted; the card may be only partly erased

THIS DESTROYS DATA. Double-check the device path before running.
"""
import argparse
import errno
import os
import stat
import sys

CHUNK = 1024 * 1024  # 1 MiB, a multiple of the 512-byte sector size
ZEROS = bytes(CHUNK)

# fsync errors that only mean "this kind of device doesn't support fsync"
FSYNC_UNSUPPORTED = {errno.EINVAL, errno.ENOTTY, getattr(errno, "ENOTSUP", -1),
                     getattr(errno, "EOPNOTSUPP", -1)}


class EraseFailure(Exception):
    pass


def device_size(dev):
    """Size of the card or image in bytes, or None if it can't be determined."""
    fd = dev.fileno()
    try:
        if stat.S_ISREG(os.fstat(fd).st_mode):
            return os.fstat(fd).st_size
    except OSError:
        pass
    try:
        if sys.platform == "darwin":
            import fcntl
            import struct
            block = struct.unpack("I", fcntl.ioctl(fd, 0x40046418, bytes(4)))[0]   # DKIOCGETBLOCKSIZE
            count = struct.unpack("Q", fcntl.ioctl(fd, 0x40086419, bytes(8)))[0]   # DKIOCGETBLOCKCOUNT
            if block and count:
                return block * count
        elif sys.platform.startswith("linux"):
            import fcntl
            import struct
            size = struct.unpack("Q", fcntl.ioctl(fd, 0x80081272, bytes(8)))[0]    # BLKGETSIZE64
            if size:
                return size
        elif os.name == "nt":
            import ctypes
            import msvcrt
            from ctypes import wintypes
            length = ctypes.c_longlong(0)
            returned = wintypes.DWORD(0)
            ok = ctypes.windll.kernel32.DeviceIoControl(
                wintypes.HANDLE(msvcrt.get_osfhandle(fd)), 0x0007405C,  # IOCTL_DISK_GET_LENGTH_INFO
                None, 0, ctypes.byref(length), 8, ctypes.byref(returned), None)
            if ok and length.value:
                return length.value
    except (OSError, ImportError, AttributeError, ValueError):
        pass
    try:
        end = os.lseek(fd, 0, os.SEEK_END)
        os.lseek(fd, 0, os.SEEK_SET)
        if end > 0:
            return end
    except OSError:
        pass
    return None


def write_all(dev, offset: int, data: bytes) -> None:
    dev.seek(offset)
    view = memoryview(data)
    while view:
        n = dev.write(view)
        if not n:
            raise EraseFailure(f"write at byte {offset:#x} wrote nothing")
        offset += n
        view = view[n:]


def erase_written(path: str, stop_after_blank_mb: int = 0, dry_run: bool = False) -> dict:
    """Zero every piece of `path` that isn't already all zeros.
    Returns a summary dict; raises EraseFailure / OSError on any error."""
    result = {"erased": 0, "scanned": 0, "size": None, "stopped_early": False}
    blank_run = 0
    mode = "rb" if dry_run else "r+b"
    with open(path, mode, buffering=0) as dev:
        size = device_size(dev)
        result["size"] = size
        if size is None:
            print("Warning: could not determine the card size; a read error will be "
                  "treated as a failure.")
        else:
            print(f"Card size: {size} bytes ({size / 2**30:.2f} GiB)")

        offset = 0
        while True:
            want = CHUNK if size is None else min(CHUNK, size - offset)
            if want <= 0:
                break
            try:
                dev.seek(offset)
                buf = dev.read(want)
            except OSError as e:
                result["scanned"] = offset
                hint = "" if size is not None else " (card size unknown, so this may be the end)"
                raise EraseFailure(f"read error at byte {offset:#x}: {e}{hint}")
            if not buf:
                if size is not None:
                    result["scanned"] = offset
                    raise EraseFailure(f"card ended early at byte {offset:#x}, expected {size} bytes")
                break

            if buf == ZEROS[:len(buf)]:
                blank_run += len(buf)
                if stop_after_blank_mb and blank_run >= stop_after_blank_mb * CHUNK:
                    offset += len(buf)
                    result["stopped_early"] = True
                    break
            else:
                blank_run = 0
                if not dry_run:
                    try:
                        write_all(dev, offset, ZEROS[:len(buf)])
                    except OSError as e:
                        result["scanned"] = offset
                        raise EraseFailure(f"write error at byte {offset:#x}: {e}")
                result["erased"] += len(buf)
            offset += len(buf)
            result["scanned"] = offset
            if (offset // CHUNK) % 64 == 0:
                print(f"\r  scanned {offset / 2**30:.2f} GiB, "
                      f"{'would erase' if dry_run else 'erased'} {result['erased'] / 2**20:.0f} MiB",
                      end="", flush=True)

        if not dry_run:
            try:
                dev.flush()
                os.fsync(dev.fileno())
            except OSError as e:
                if e.errno not in FSYNC_UNSUPPORTED:
                    raise EraseFailure(f"could not flush writes to the card: {e}")
                # Raw devices that don't support fsync are written unbuffered anyway.
    print(f"\r  scanned {result['scanned'] / 2**30:.2f} GiB, "
          f"{'would erase' if dry_run else 'erased'} {result['erased'] / 2**20:.0f} MiB")
    return result


def main() -> int:
    p = argparse.ArgumentParser(description="Erase all written blocks on an SD card.")
    p.add_argument("device", help="raw device path, e.g. /dev/sdX, /dev/rdiskN, \\\\.\\PhysicalDriveN")
    p.add_argument("--stop-after-blank", type=int, default=0, metavar="MB",
                   help="stop after this many consecutive all-zero MiB (default: scan whole card)")
    p.add_argument("--dry-run", action="store_true", help="scan only, write nothing")
    p.add_argument("--yes", action="store_true", help="skip confirmation")
    args = p.parse_args()

    if not args.dry_run and not args.yes:
        print(f"About to ERASE all data on {args.device}. This cannot be undone.")
        if input("Type the device path again to confirm: ").strip() != args.device:
            print("Aborted.")
            return 1

    try:
        result = erase_written(args.device, args.stop_after_blank, args.dry_run)
    except PermissionError:
        print("\nPermission denied: run as root/Administrator and unmount the card first.")
        print("On Windows, remove the card's volume first with diskpart "
              "(select disk N, then clean); see the README.")
        return 1
    except EraseFailure as e:
        print(f"\nERROR: {e}")
        if not args.dry_run:
            print("The card is only partly erased. Fix the problem and run the erase again.")
        print("macOS: run 'diskutil unmountDisk /dev/diskN' first. Windows: remove the card's "
              "volume with diskpart (select disk N, then clean); see the README.")
        return 1
    except OSError as e:
        print(f"\nError on {args.device}: {e}")
        print("macOS: run 'diskutil unmountDisk /dev/diskN' first.")
        print("Windows: Windows blocks writes to a disk with a mounted volume; remove it with "
              "diskpart (select disk N, then clean); see the README.")
        return 1

    verb = "would erase" if args.dry_run else "erased"
    scope = (f"stopped after {args.stop_after_blank} MiB of zeros, as requested"
             if result["stopped_early"] else "whole card scanned")
    print(f"Done: {verb} {result['erased'] / 2**20:.0f} MiB ({scope}).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
