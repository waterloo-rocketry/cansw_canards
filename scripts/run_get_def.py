from pathlib import Path

def validate_structure(path, identifer, line_number, line):

    # Checking for a valid start
    if (not line.startswith(identifer)):
        raise ValueError(
            f"Invalid definition at {path}:{line_number}: expected the entry to start "
            f"with '{identifer}(' (the expected macro name), but found {line!r}."
        )

    # Checking for valid end
    if (line[-1] != ")"):
        raise ValueError(
            f"Invalid definition at {path}:{line_number}: entry {line!r} must end "
            f"with a closing parenthesis ')'."
        )

    return True

def validate_enum_fields(path, line_number, enum_name, enum_names_dict, enum_value, enum_values_dict):
    valid_values = "0123456789abcdef"

    # Checking if the value is a valid hexadecimal value
    if (not enum_value.startswith("0x")):
        raise ValueError(
            f"Invalid enum value at {path}:{line_number}: {enum_value!r} must start "
            f"with the hexadecimal prefix '0x'."
        )

    # Checking if the enum value contains only valid values for hexadecimal.
    for value in enum_value.removeprefix("0x"):
        if value not in valid_values:
            raise ValueError(
                f"Invalid enum value at {path}:{line_number}: {enum_value!r} contains "
                f"invalid hexadecimal character {value!r}; valid digits are 0-9 and a-f."
            )

    # Duplication enum_name check
    if (enum_name in enum_names_dict):
        raise ValueError(
            f"Duplicate enum name at {path}:{line_number}: {enum_name!r} was already "
            f"defined at line {enum_names_dict[enum_name]}."
        )

    # Duplication enum_value check
    if (enum_value in enum_values_dict):
        raise ValueError(
            f"Duplicate enum value at {path}:{line_number}: {enum_value!r} was already "
            f"assigned at line {enum_values_dict[enum_value]}."
        )

    return True

def parse_definitions(path, identifer, expected_num_fields):
    all_def_lines = path.read_text(encoding="utf-8").splitlines()

    enum_entries = []
    enum_names_dict = {}
    enum_values_dict = {}

    for line_number, line in enumerate(all_def_lines, start=1):
        if (line == "" or line.startswith("//") or line.startswith("/*") or line.startswith("*")):
            continue

        # Validating the overall structure of the current line
        validate_structure(path, identifer, line_number, line)

        # Splitting line into its fields
        line_fields = (line.removeprefix(identifer + "(").removesuffix(")")).split(",", expected_num_fields - 1)
        for field_num in range(len(line_fields)):
            line_fields[field_num] = line_fields[field_num].strip()

        if (len(line_fields) != expected_num_fields):
            raise ValueError(
                f"Invalid field count at {path}:{line_number}: expected at least "
                f"{expected_num_fields} fields, but found {len(line_fields)} in {line!r}."
            )

        enum_name = line_fields[0]
        enum_value = line_fields[1].lower()

        validate_enum_fields(path, line_number, enum_name, enum_names_dict, enum_value, enum_values_dict)

        # Adding enums to their respective dictionaries for duplication checking
        enum_names_dict[enum_name] = line_number
        enum_values_dict[enum_value] = line_number
        enum_entries.append((enum_name, enum_value))

    return enum_entries

def write_definitions(definitions_path, events_list, files_list, tasks_list):
    lines = [
        "#ifndef LOG__DEFINITIONS_H",
        "#define LOG__DEFINITIONS_H",
        "",
        "#ifdef __cplusplus",
        "#define LOG_STATIC_ASSERT static_assert",
        "#else",
        "#define LOG_STATIC_ASSERT _Static_assert",
        "#endif",
        "",
        "/* BEGIN GENERATED ERROR ENUM",
        "    A standard error value is split in two parts:",
        "    First two digits represent the file name",
        "    Next two digits count up from 0x00",
        "*/",
        "typedef enum __attribute__((packed)) {",
        "\tLOG_ERRORS_ERROR = 0x00,"
    ]

    for enum_name, enum_value in events_list:
        lines.append(f"\tLOG_ERRORS_{enum_name} = {enum_value},")

    lines.extend([
        "\tLOG_ERRORS_MAX_SIZE = 0xffff",
        "} log_error_id_t;",
        "",
        'LOG_STATIC_ASSERT(sizeof(log_error_id_t) == 2, "log_file_error_t must be 2 bytes");',
        "/* END GENERATED ERROR ENUM */",
        "",
        "/* BEGIN GENERATED FILE ENUM */",
        "typedef enum __attribute__((packed)) {",
        "\tLOG_FILES_ERROR = 0x00,"
    ])

    for enum_name, enum_value in files_list:
        lines.append(f"\t{enum_name} = {enum_value},")

    lines.extend([
        "\tLOG_FILES_SIZE_PIN = 0xff",
        "} log_file_id_t;",
        "",
        'LOG_STATIC_ASSERT(sizeof(log_file_id_t) == 1, "log_file_id_t must be 1 byte");',
        "/* END GENERATED FILE ENUM */",
        "",
        "/* BEGIN GENERATED TASK ENUM",
        "    Tasks increment by one starting from 0",
        "*/",
        "typedef enum __attribute__((packed)) {",
        "\tLOG_TASK_ERROR = 0x00,"
    ])

    for enum_name, enum_value in tasks_list:
        lines.append(f"\t{enum_name} = {enum_value},")

    lines.extend([
        "\tLOG_TASK_MAX_SIZE = 0xff",
        "} log_task_id_t;",
        "",
        'LOG_STATIC_ASSERT(sizeof(log_task_id_t) <= 1, "log_task_id_t must be at 1 byte");',
        "/* END GENERATED TASK ENUM */",
        "",
        "#endif"
    ])

    definitions_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

def main():
    root = Path(__file__).resolve().parents[1]
    dictionary_path = root / "log_dictionary"
    definitions_path = (
        root / "src" / "application" / "logger" / "log_definitions.h"
    )

    # Parsing log_events.def
    events_path = dictionary_path / "log_events.def"
    events_list = parse_definitions(events_path, "EVENT", expected_num_fields = 4)

    # Parsing log_events.def
    files_path = dictionary_path / "log_files.def"
    files_list = parse_definitions(files_path, "FILE_ID", expected_num_fields = 3)

    # Parsing log_events.def
    tasks_path = dictionary_path / "log_tasks.def"
    tasks_list = parse_definitions(tasks_path, "TASK_ID", expected_num_fields = 3)

    write_definitions(definitions_path, events_list, files_list, tasks_list)


if __name__ == "__main__":
    main()