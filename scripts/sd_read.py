#!/usr/bin/env python3
"""Read an SD card's raw contents (read-only; never writes to the card).

Default: scan the card and list every region that contains data
(anything that is not all 0x00 or all 0xFF), then hexdump the start of
the first region.

Usage (needs root / Administrator):
    sudo python3 sd_read.py /dev/rdiskN                  scan + summary
    sudo python3 sd_read.py /dev/rdiskN --hexdump 0      hexdump from byte offset 0
    sudo python3 sd_read.py /dev/rdiskN --hexdump 0x100000 --length 1024
    sudo python3 sd_read.py /dev/rdiskN --dump out.bin   copy written data to a file
                                                         (from offset 0 to end of last
                                                         written region)
    --stop-after-blank MB    stop scanning after MB consecutive blank MiB
"""
import argparse
import sys

SECTOR = 512
CHUNK = 1024 * 1024  # 1 MiB
ZERO_SEC = bytes(SECTOR)
ONES_SEC = b"\xff" * SECTOR
ZERO_CHUNK = bytes(CHUNK)
ONES_CHUNK = b"\xff" * CHUNK


def read_at(dev, offset: int, length: int) -> bytes:
    """Sector-aligned read (raw devices require aligned offsets and sizes)."""
    start = offset - offset % SECTOR
    end = offset + length
    end += -end % SECTOR
    dev.seek(start)
    data = dev.read(end - start)
    return data[offset - start: offset - start + length]


def hexdump(data: bytes, base: int = 0) -> None:
    prev, skipping = None, False
    for i in range(0, len(data), 16):
        row = data[i:i + 16]
        if row == prev:
            if not skipping:
                print("*")
                skipping = True
            continue
        prev, skipping = row, False
        hexpart = " ".join(f"{b:02x}" for b in row)
        text = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        print(f"{base + i:010x}  {hexpart:<47}  |{text}|")


def scan(dev, stop_after_blank_mb: int = 0):
    """Return (regions, bytes_scanned). regions = list of [start, end) byte offsets
    holding data, at 512-byte sector resolution."""
    regions = []
    offset = 0
    blank_run = 0
    dev.seek(0)
    while True:
        try:
            buf = dev.read(CHUNK)
        except OSError as e:
            print(f"\n  read stopped at offset {offset:#x}: {e}")
            break
        if not buf:
            break
        n = len(buf)
        if buf == ZERO_CHUNK[:n] or buf == ONES_CHUNK[:n]:
            blank_run += n
            if stop_after_blank_mb and blank_run >= stop_after_blank_mb * CHUNK:
                offset += n
                break
        else:
            blank_run = 0
            for s in range(0, n, SECTOR):
                sec = buf[s:s + SECTOR]
                if sec != ZERO_SEC[:len(sec)] and sec != ONES_SEC[:len(sec)]:
                    a, b = offset + s, offset + s + len(sec)
                    if regions and regions[-1][1] == a:
                        regions[-1][1] = b
                    else:
                        regions.append([a, b])
        offset += n
        if (offset // CHUNK) % 64 == 0:
            print(f"\r  scanned {offset / 2**30:.2f} GiB...", end="", flush=True)
    print(f"\r  scanned {offset / 2**30:.2f} GiB ({offset} bytes)")
    return regions, offset


def main() -> int:
    p = argparse.ArgumentParser(description="Read raw contents of an SD card (read-only).")
    p.add_argument("device", help="e.g. /dev/rdiskN, /dev/sdX, \\\\.\\PhysicalDriveN")
    p.add_argument("--hexdump", metavar="OFFSET", type=lambda s: int(s, 0),
                   help="hexdump starting at this byte offset (decimal or 0x hex) instead of scanning")
    p.add_argument("--length", type=lambda s: int(s, 0), default=512,
                   help="bytes to hexdump (default 512)")
    p.add_argument("--dump", metavar="FILE", help="copy data from offset 0 through the last written region to FILE")
    p.add_argument("--stop-after-blank", type=int, default=0, metavar="MB",
                   help="stop scanning after this many consecutive blank MiB")
    args = p.parse_args()

    try:
        dev = open(args.device, "rb", buffering=0)
    except PermissionError:
        print("Permission denied: run with sudo / as Administrator.")
        return 1

    with dev:
        if args.hexdump is not None:
            data = read_at(dev, args.hexdump, args.length)
            if not data:
                print("No data returned (offset past end of device?).")
                return 1
            hexdump(data, args.hexdump)
            return 0

        regions, scanned = scan(dev, args.stop_after_blank)
        if scanned == 0:
            print("Could not read anything from the device. Check the path and that the card is inserted.")
            return 1
        total = sum(b - a for a, b in regions)
        if not regions:
            print("Card is blank: every scanned byte is 0x00 or 0xFF.")
            return 0

        print(f"Found data: {total} bytes ({total / 2**20:.2f} MiB) in {len(regions)} region(s):")
        for a, b in regions[:50]:
            print(f"  {a:#012x} - {b:#012x}   sectors {a // SECTOR}-{b // SECTOR - 1}   {b - a} bytes")
        if len(regions) > 50:
            print(f"  ... and {len(regions) - 50} more")

        a, b = regions[0]
        print(f"\nFirst {min(256, b - a)} bytes of first region:")
        hexdump(read_at(dev, a, min(256, b - a)), a)

        if args.dump:
            end = regions[-1][1]
            dev.seek(0)
            with open(args.dump, "wb") as out:
                left = end
                while left > 0:
                    buf = dev.read(min(CHUNK, left + (-left % SECTOR)))
                    if not buf:
                        break
                    out.write(buf[:left])
                    left -= len(buf)
            print(f"\nWrote {end} bytes to {args.dump}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
