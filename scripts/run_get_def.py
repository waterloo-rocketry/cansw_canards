# NOTE THIS FILE IS CURRENTLY HERE FOR TESTING PURPOSES
import re
from pathlib import Path


def main():
    repo_root = Path(__file__).resolve().parents[1]
    events_path = repo_root / "log_dictionary" / "log_events.def"
    definitions_path = repo_root / "src" / "application" / "logger" / "log_definitions.h"

    events = []
    for line_number, line in enumerate(events_path.read_text().splitlines(), start=1):
        if not line.lstrip().startswith("EVENT("):
            continue

        match = re.fullmatch(
            r'\s*EVENT\(\s*(\w+)\s*,\s*(\d+)\s*,\s*\d+\s*,\s*".*"\s*\)\s*',
            line,
        )
        if match is None:
            raise ValueError(f"Malformed EVENT entry at {events_path}:{line_number}")

        name, event_id = match.groups()
        events.append((name, event_id))

    if not events:
        raise ValueError(f"No EVENT entries found in {events_path}")

    event_names = [name for name, _ in events]
    event_ids = [event_id for _, event_id in events]
    if len(event_names) != len(set(event_names)):
        raise ValueError(f"Duplicate EVENT name found in {events_path}")
    if len(event_ids) != len(set(event_ids)):
        raise ValueError(f"Duplicate EVENT id found in {events_path}")

    files_path = repo_root / "log_dictionary" / "log_files.def"
    files = []
    for line_number, line in enumerate(files_path.read_text().splitlines(), start=1):
        if not line.lstrip().startswith("FILE_ID("):
            continue

        match = re.fullmatch(
            r'\s*FILE_ID\(\s*(\w+)\s*,\s*(\d+)\s*,\s*"(?:\\.|[^"\\])*"\s*\)\s*',
            line,
        )
        if match is None:
            raise ValueError(f"Malformed FILE_ID entry at {files_path}:{line_number}")

        name, file_id = match.groups()
        files.append((name, file_id))

    if not files:
        raise ValueError(f"No FILE_ID entries found in {files_path}")

    file_names = [name for name, _ in files]
    file_ids = [file_id for _, file_id in files]
    if len(file_names) != len(set(file_names)):
        raise ValueError(f"Duplicate FILE_ID name found in {files_path}")
    if len(file_ids) != len(set(file_ids)):
        raise ValueError(f"Duplicate FILE_ID id found in {files_path}")

    header = definitions_path.read_text()
    event_enum_pattern = re.compile(
        r"(?:"
        r"typedef enum\s*\{\s*"
        r"#define EVENT\(name, id, num_args, message\) LOG_EVENTS_##name = id,\s*"
        r'#include "log_events\.def"\s*'
        r"#undef EVENT\s*"
        r"\}\s*log_event_id_t;"
        r"|/\* BEGIN GENERATED EVENT ENUM \*/\s*"
        r"typedef enum\s*\{(?:\s*LOG_EVENTS_\w+\s*=\s*\d+,)*\s*\}"
        r"\s*log_event_id_t;\s*/\* END GENERATED EVENT ENUM \*/"
        r"|typedef enum\s*\{(?:\s*LOG_EVENTS_\w+\s*=\s*\d+,)+\s*\}"
        r"\s*log_event_id_t;"
        r")",
        re.MULTILINE,
    )
    replacement = (
        "/* BEGIN GENERATED EVENT ENUM */\n"
        "typedef enum {\n"
        + "".join(f"\tLOG_EVENTS_{name} = {event_id},\n" for name, event_id in events)
        + "} log_event_id_t;\n"
        "/* END GENERATED EVENT ENUM */"
    )
    updated_header, replacements = event_enum_pattern.subn(replacement, header)
    if replacements != 1:
        raise ValueError(
            f"Expected one event enum macro block in {definitions_path}, found {replacements}"
        )

    file_enum_pattern = re.compile(
        r"(?:"
        r"typedef enum\s*\{\s*"
        r"#define FILE_ID\(name, id, file_path\) LOG_FILES_##name = id,\s*"
        r'#include "log_files\.def"\s*'
        r"#undef FILE_ID\s*"
        r"\}\s*log_file_id_t;"
        r"|/\* BEGIN GENERATED FILE ENUM \*/\s*"
        r"typedef enum\s*\{(?:\s*LOG_FILES_\w+\s*=\s*\d+,)*\s*\}"
        r"\s*log_file_id_t;\s*/\* END GENERATED FILE ENUM \*/"
        r"|typedef enum\s*\{(?:\s*LOG_FILES_\w+\s*=\s*\d+,)+\s*\}"
        r"\s*log_file_id_t;"
        r")",
        re.MULTILINE,
    )
    file_enum_replacement = (
        "/* BEGIN GENERATED FILE ENUM */\n"
        "typedef enum {\n"
        + "".join(
            f"\tLOG_FILES_{name} = {file_id},\n" for name, file_id in files
        )
        + "} log_file_id_t;\n"
        "/* END GENERATED FILE ENUM */"
    )
    updated_header, replacements = file_enum_pattern.subn(
        file_enum_replacement, updated_header
    )
    if replacements != 1:
        raise ValueError(
            f"Expected one file enum macro block in {definitions_path}, found {replacements}"
        )

    definitions_path.write_text(updated_header)


if __name__ == "__main__":
    main()