#!/usr/bin/env python3
"""Extract logged files from a raw SD card written by logging_formatter.c.
Read-only: never writes to the card.

Chunk layout (CHUNK_SIZE = 32768 bytes, multi-byte fields big-endian),
matching logging_formatting_write():

    bytes 0-3        constant header 0x43 0x6B 0xCF 0xFF
    bytes 4-5        bits 15..3 = 13-bit file identifier (one per boot)
                     bits  2..0 = data type (0 DATA, 1 ERROR, 2 TEXT)
    bytes 6-9        32-bit chunk index within the file (0, 1, 2, ...)
    bytes 10-32765   payload (32756 bytes)
    bytes 32766-7    CRC-16/MODBUS of bytes 0-32765 (header + payload),
                     high byte first

Chunk 0 is reserved by the firmware and is skipped.
Chunks with the same (identifier, type) are joined in chunk-index order into
file_<identifier>_<TYPE>.bin (.txt for TEXT).

DATA payloads hold whole 32-byte records (1023 x 32 = 32736 bytes) followed by
20 bytes of padding; the padding is removed so records stay aligned across
chunks. TEXT files have trailing zero padding removed.

Memory use stays small on large cards: the first pass keeps only where each
chunk is on the card, and the second pass re-reads chunks to write the files.

Exit status:
    0  everything read and extracted cleanly
    1  error: could not open/read the card or write the output, or the read
       ended before the end of the card
    2  extraction finished, but some chunks were bad (CRC failure, unknown
       header) or files have missing chunks

Usage:
    sudo python3 sd_extract.py /dev/rdiskN -o sd_files
Options:
    --data-record-size N size of one DATA record (default 32; 0 = keep the
                         whole payload, padding included)
    --crc-start N        first byte the CRC covers (default 0 = header + payload;
                         10 for old firmware that CRC'd the payload only)
    --crc-zero-pad N     the firmware CRC'd N bytes past byte 32765 that the
                         footer then overwrote; assume they were zero
    --crc-order big|little   byte order of the stored CRC (default big)
    --keep-bad           also write chunks that fail the CRC
    --stop-after-blank N stop after N consecutive blank chunks (default 32,
                         0 = read the whole card)
"""
import argparse
import os
import stat
import sys

CHUNK_SIZE = 32768
HEADER = bytes([0x43, 0x6B, 0xCF, 0xFF])
PAYLOAD_START = 10
CRC_SIZE = 2
PAYLOAD_END = CHUNK_SIZE - CRC_SIZE          # 32766
PAYLOAD_SIZE = PAYLOAD_END - PAYLOAD_START   # 32756
FIRST_CHUNK = 1                              # chunk 0 is reserved
TYPE_DATA, TYPE_ERROR, TYPE_TEXT = 0, 1, 2
TYPE_NAMES = {TYPE_DATA: "DATA", TYPE_ERROR: "ERROR", TYPE_TEXT: "TEXT"}

EXIT_OK, EXIT_ERROR, EXIT_DATA_PROBLEMS = 0, 1, 2


class ReadFailure(Exception):
    pass


def _make_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        table.append(crc)
    return table


CRC_TABLE = _make_table()


def crc_continue(crc: int, data: bytes) -> int:
    for b in data:
        crc = (crc >> 8) ^ CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def crc16_modbus(data: bytes) -> int:
    return crc_continue(0xFFFF, data)


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


def read_chunk(dev, chunk_no: int, size):
    """Read one whole chunk. Returns None at the true end of the card; raises
    ReadFailure on an I/O error or if the card ends earlier than its size says."""
    offset = chunk_no * CHUNK_SIZE
    if size is not None and offset + CHUNK_SIZE > size:
        return None  # no whole chunk left before the end of the card
    try:
        dev.seek(offset)
        data = b""
        while len(data) < CHUNK_SIZE:
            part = dev.read(CHUNK_SIZE - len(data))
            if not part:
                break
            data += part
    except OSError as e:
        where = f"chunk {chunk_no} (byte {offset:#x})"
        if size is None:
            raise ReadFailure(f"read error at {where}: {e} (card size unknown, so this may be "
                              f"the end of the card or a real error)")
        raise ReadFailure(f"read error at {where}: {e}")
    if len(data) < CHUNK_SIZE:
        if size is None and not data:
            return None  # end of card
        raise ReadFailure(f"card ended early at chunk {chunk_no} (byte {offset + len(data):#x})"
                          + (f", expected {size} bytes" if size is not None else ""))
    return data


def is_blank(chunk: bytes) -> bool:
    return chunk.count(0) == len(chunk) or chunk.count(0xFF) == len(chunk)


def diagnose(chunks):
    """Try the plausible CRC coverages; return the ones that match every chunk."""
    results = None
    for chunk in chunks:
        stored = {o: int.from_bytes(chunk[PAYLOAD_END:], o) for o in ("big", "little")}
        found = set()
        for start in (0, 4, 6, 10):
            crc = crc16_modbus(chunk[start:PAYLOAD_END])
            for pad in range(0, 9):
                if pad:
                    crc = crc_continue(crc, b"\0")
                for order, value in stored.items():
                    if crc == value:
                        found.add((start, pad, order))
        results = found if results is None else results & found
    return results or set()


def parse_chunk(chunk: bytes, crc_start: int, crc_pad: int, crc_order: str):
    word = int.from_bytes(chunk[4:6], "big")
    covered = chunk[crc_start:PAYLOAD_END] + bytes(crc_pad)
    stored = int.from_bytes(chunk[PAYLOAD_END:], crc_order)
    computed = crc16_modbus(covered)
    return {
        "header_ok": chunk[:4] == HEADER,
        "header": chunk[:4],
        "file_id": word >> 3,
        "type": word & 0b111,
        "index": int.from_bytes(chunk[6:10], "big"),
        "crc_ok": stored == computed,
        "crc_stored": stored,
        "crc_computed": computed,
    }


def payload_for(chunk: bytes, dtype: int, data_len: int) -> bytes:
    payload = chunk[PAYLOAD_START:PAYLOAD_END]
    if dtype == TYPE_DATA:
        payload = payload[:data_len]
    return payload


def main() -> int:
    p = argparse.ArgumentParser(description="Extract logged files from a raw SD card.")
    p.add_argument("device", help="raw device (/dev/rdiskN, /dev/sdX, \\\\.\\PhysicalDriveN) "
                                  "or a card image file")
    p.add_argument("-o", "--out", default="sd_files", help="output folder (default: sd_files)")
    p.add_argument("--data-record-size", type=int, default=32, metavar="N",
                   help="DATA record size; payload is trimmed to whole records "
                        "(default 32, 0 = keep the full payload)")
    p.add_argument("--crc-start", type=int, default=0,
                   help="first byte covered by the CRC (default 0; 10 for old firmware)")
    p.add_argument("--crc-zero-pad", type=int, default=0,
                   help="zero bytes the firmware CRC'd after byte 32765 (default 0)")
    p.add_argument("--crc-order", choices=["big", "little"], default="big",
                   help="byte order of the stored CRC (default big)")
    p.add_argument("--keep-bad", action="store_true", help="include chunks that fail the CRC")
    p.add_argument("--stop-after-blank", type=int, default=32, metavar="N",
                   help="stop after N consecutive blank chunks (0 = read whole card)")
    args = p.parse_args()

    if args.data_record_size < 0 or args.data_record_size > PAYLOAD_SIZE:
        p.error(f"--data-record-size must be between 0 and {PAYLOAD_SIZE}")
    if args.data_record_size:
        data_len = PAYLOAD_SIZE // args.data_record_size * args.data_record_size
    else:
        data_len = PAYLOAD_SIZE

    try:
        dev = open(args.device, "rb", buffering=0)
    except PermissionError:
        print("Permission denied: run with sudo (macOS/Linux) or as Administrator (Windows).")
        return EXIT_ERROR
    except OSError as e:
        print(f"Could not open {args.device}: {e}")
        print("macOS 'Resource busy': run diskutil unmountDisk /dev/diskN")
        print("Windows: run the terminal as Administrator and use \\\\.\\PhysicalDriveN")
        return EXIT_ERROR

    # (file_id, type) -> {index: (chunk_no, crc_ok)}; only locations, never payloads
    files = {}
    stats = {"read": 0, "blank": 0, "bad_header": 0, "bad_crc": 0, "good": 0, "duplicates": 0}
    bad_samples = []
    stopped_early = False

    with dev:
        size = device_size(dev)
        if size is None:
            print("Warning: could not determine the card size; a read error will be "
                  "treated as a failure.")
        else:
            print(f"Card size: {size} bytes ({size / 2**30:.2f} GiB)")

        # Pass 1: find every chunk and remember where it is.
        blank_run = 0
        chunk_no = FIRST_CHUNK
        try:
            while True:
                chunk = read_chunk(dev, chunk_no, size)
                if chunk is None:
                    break
                this_chunk = chunk_no
                chunk_no += 1
                stats["read"] += 1

                if is_blank(chunk):
                    stats["blank"] += 1
                    blank_run += 1
                    if args.stop_after_blank and blank_run >= args.stop_after_blank:
                        stopped_early = True
                        break
                    continue
                blank_run = 0

                c = parse_chunk(chunk, args.crc_start, args.crc_zero_pad, args.crc_order)
                if not c["header_ok"]:
                    stats["bad_header"] += 1
                    note = ""
                    if chunk[:3] in (b"\xeb\x58\x90", b"\xeb\x3c\x90", b"\xeb\x76\x90"):
                        note = " (FAT/exFAT boot sector: card was never erased)"
                    print(f"  chunk {this_chunk}: header {c['header'].hex(' ')}, skipped{note}")
                    continue

                tname = TYPE_NAMES.get(c["type"], f"TYPE{c['type']}")
                if not c["crc_ok"]:
                    stats["bad_crc"] += 1
                    if len(bad_samples) < 5:
                        bad_samples.append(chunk)
                    print(f"  chunk {this_chunk}: CRC mismatch (stored 0x{c['crc_stored']:04X}, "
                          f"computed 0x{c['crc_computed']:04X}), file {c['file_id']} {tname} "
                          f"index {c['index']}" + ("" if args.keep_bad else ", skipped"))
                    if not args.keep_bad:
                        continue
                else:
                    stats["good"] += 1

                entry = files.setdefault((c["file_id"], c["type"]), {})
                if c["index"] in entry:
                    stats["duplicates"] += 1
                    print(f"  chunk {this_chunk}: duplicate index {c['index']} for file "
                          f"{c['file_id']} {tname}, keeping the later one")
                entry[c["index"]] = (this_chunk, c["crc_ok"])
        except ReadFailure as e:
            print(f"\nERROR: {e}")
            print(f"Stopped after {stats['read']} chunks. Nothing was extracted, because the "
                  f"rest of the card could not be read.")
            return EXIT_ERROR

        how = (f"stopped after {args.stop_after_blank} blank chunks in a row"
               if stopped_early else "reached the end of the card")
        print(f"\nRead {stats['read']} chunks (from chunk {FIRST_CHUNK}, {how}): "
              f"{stats['good']} good, {stats['bad_crc']} bad CRC, "
              f"{stats['bad_header']} wrong header, {stats['blank']} blank")

        if bad_samples and stats["good"] == 0:
            print("\nEvery CRC failed. Testing which bytes the firmware's CRC covers...")
            matches = diagnose(bad_samples)
            if matches:
                for start, pad, order in sorted(matches):
                    print(f"  match: --crc-start {start} --crc-zero-pad {pad} --crc-order {order}")
                    if pad:
                        print(f"    (the firmware CRC'd {PAYLOAD_END - start + pad} bytes, so the "
                              f"payload ran {pad} byte(s) into the footer and the CRC overwrote them)")
                print("Re-run with those options.")
            else:
                print("  No coverage matched. The bytes on the card are probably not the bytes the")
                print("  CPU computed the CRC over (e.g. D-cache not cleaned before the SD DMA write).")

        problems = stats["bad_crc"] + stats["bad_header"] + stats["duplicates"]

        if not files:
            print("No files found.")
            return EXIT_DATA_PROBLEMS if problems else EXIT_OK

        # Pass 2: re-read each chunk from the card and write the files one chunk at a time.
        try:
            os.makedirs(args.out, exist_ok=True)
        except OSError as e:
            print(f"ERROR: could not create {args.out}: {e}")
            return EXIT_ERROR

        print(f"\nWriting {len(files)} file(s) to {args.out}/")
        for (file_id, dtype), chunks in sorted(files.items()):
            tname = TYPE_NAMES.get(dtype, f"TYPE{dtype}")
            ext = "txt" if dtype == TYPE_TEXT else "bin"
            name = f"file_{file_id:04d}_{tname}.{ext}"
            path = os.path.join(args.out, name)
            indices = sorted(chunks)
            written = 0
            held_zeros = 0  # TEXT: trailing zeros held back until more text follows
            try:
                with open(path, "wb") as out:
                    for idx in indices:
                        chunk_no, was_ok = chunks[idx]
                        chunk = read_chunk(dev, chunk_no, size)
                        if chunk is None:
                            raise ReadFailure(f"chunk {chunk_no} disappeared on re-read")
                        if was_ok and not parse_chunk(chunk, args.crc_start, args.crc_zero_pad,
                                                      args.crc_order)["crc_ok"]:
                            raise ReadFailure(f"chunk {chunk_no} passed its CRC on the first read "
                                              f"but failed on the second: the card is returning "
                                              f"inconsistent data")
                        payload = payload_for(chunk, dtype, data_len)
                        if dtype == TYPE_TEXT:
                            text = payload.rstrip(b"\x00")
                            if text:
                                out.write(bytes(held_zeros) + text)
                                written += held_zeros + len(text)
                                held_zeros = len(payload) - len(text)
                            else:
                                held_zeros += len(payload)
                        else:
                            out.write(payload)
                            written += len(payload)
            except ReadFailure as e:
                print(f"\nERROR while writing {name}: {e}")
                print(f"{name} is incomplete.")
                return EXIT_ERROR
            except OSError as e:
                print(f"\nERROR: could not write {path}: {e}")
                return EXIT_ERROR

            missing = sorted(set(range(indices[-1] + 1)) - set(indices))
            if missing:
                problems += 1
            note = (f"   <-- missing index {missing[:10]}{'...' if len(missing) > 10 else ''}"
                    if missing else "")
            print(f"  {name:<28} {len(indices):>5} chunk(s) {written:>10} bytes{note}")

    if problems:
        print("\nFinished with problems (see above): exit status 2.")
        return EXIT_DATA_PROBLEMS
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
