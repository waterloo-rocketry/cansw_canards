# Log Dictionary

These definition files describe log events, source files, and FreeRTOS tasks. They are used to generate `log_definitions.h` and decode binary error logs.

## `log_errors.def`

Defines the events the program can log.

Each entry has the form:

```c
EVENT(enum_name, event_id, num_args, message)
```

- `enum_name`: Identifier used in code. The generated enum name is prefixed with `LOG_ERROR_`.
- `event_id`: 16-bit hexadecimal ID. The high byte is the source file ID; the low byte is the event code.
- `num_args`: Number of format arguments in the message.
- `message`: `printf`-style message string.

```text
0x  FF EE
    |  |
    |  +-- Event code (1 byte), counted within the source file
    +----- File ID (1 byte), defined in `log_files.def`
```

For example, `0x1B06` refers to file ID `0x1B` (`uart.c`) and event code `0x06`.

To add an event, choose an unused event code for its source file, set `num_args` to match the message placeholders, then use the generated `LOG_ERROR_<enum_name>` with `LOG_ERROR()` in code. Run `python scripts/run_get_def.py` to regenerate `log_definitions.h`.

## `log_files.def`

Defines the source files that can emit logs.

```c
FILE_ID(enum_file_name, file_id, file_path)
```

Add a unique file ID and source path for each file that uses `LOG_ERROR()`. In that source file, define `LOG_FILE_ID` to the corresponding enum; this supplies the high byte of its event IDs. Run `python scripts/run_get_def.py` to regenerate `log_definitions.h`.

## `log_tasks.def`

Defines the FreeRTOS tasks that can emit logs. Task ID `0x00` is reserved for `LOG_TASK_ERROR`; other IDs start at `0x01`. Each task's ID is assigned in `init.c` with `vTaskSetTaskNumber()`.

```c
TASK_ID(enum_task_name, task_id, task_name)
```

`task_name` is the name string passed to `xTaskCreate()`.
When adding a task, give it a unique ID and assign its generated enum to the task handle with `vTaskSetTaskNumber()`. Run `python scripts/run_get_def.py` to regenerate `log_definitions.h`.

## Scripts

- `run_get_def.py` generates `log_definitions.h` from the three definition files.
- `error_log_decoder.py` decodes a binary error log using the definition files. Pass the log path and this folder's `log_errors.def` with `--events_def`; use `--files_def` or `--tasks_def` to override their defaults.
