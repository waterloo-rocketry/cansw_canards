# ---- Work in progress of error log decoding script ----




import re
import argparse
from pathlib import Path
from collections import namedtuple
# Constants for file paths
REPO_ROOT = Path(__file__).resolve().parents[1]
DEF_DIR = REPO_ROOT / "log_dictionary"


# .def file regex formats
COMMENTS = r'^(?:\s*/\*.*?\*/)*\s*'   # To account for /* ... */ comments before an entry
EVENT_PATTERN = re.compile(COMMENTS + r'EVENT\(\s*(\w+)\s*,\s*(0x[0-9a-fA-F]+|\d+)\s*,\s*(\d+)\s*,\s*"(.*)"\s*\)')
FILE_PATTERN = re.compile(COMMENTS + r'FILE_ID\(\s*(\w+)\s*,\s*(0x[0-9a-fA-F]+|\d+)\s*,\s*"(.*)"\s*\)')


# nametuple for event dictionary
Event = namedtuple("Event", ["num_args", "message"])


# .bin delimiters
VALUE_DELIM = "|"
ARG_DELIM = ","


# Read event def file, create + return dictionary of all error log strings
def get_event_dict(args):
    # Create dictionary that will store event info
    event_info = {}


    # Loop through event def file and store relevant lines
    with open(args.events_def, 'r', encoding='utf-8') as file:
        for line in file:
            # If line doesn't match event pattern, discard
            m = EVENT_PATTERN.match(line)
            if m is None:
                continue
            enum_name, event_id, num_args, message = m.groups()
            event_info[int(event_id, 0)] = Event(int(num_args), message)


    # TEMP: Check if dict is defined properly
        #for event_id, event in list(event_info.items())[:5]:
            #print(event_id, event)


    return event_info


# Read file def file, create + return dictionary of all error log strings
def get_file_dict(args):
    # Create dictionary that will store event info
    file_info = {}
   
    # Loop through file def file and store relevant lines
    with open(args.files_def, 'r', encoding='utf-8') as file:
        for line in file:
            m = FILE_PATTERN.match(line)
            if m is None:
                continue
            enum_name, file_id, path = m.groups()
            file_info[int(file_id, 0)] = path


    # TEMP: Check if dict is defined properly
        #for file_id, path in list(file_info.items())[:5]:
            #print(file_id, path)        


    return file_info




def read_bin_file(args, event_dict, file_dict):
    lines = []
    with open(args.logs_bin) as file:
        # lambda is mini function, repeats reading 32 bytes at a time until end of file
        for record in iter(lambda: file.read(32), b""):
            # Print to console if partial record encountered
            if len(record) < 32:
                print(f"INFO: partial record ({len(record)} bytes) encountered.")
            # Call function that will take binary line and translate to string message
            line = translate_log(record, event_dict, file_dict)
            if line is not None:
                lines.append(line)
    # Return list of translated lines
    return lines


def translate_log(chunk, event_dict, file_dict):
    # TODO:
        # Split log by delimiter
        # Store each section in variable
        # Use dicts to convert each section to string
            # Plug #args, event string, and args into insert_args
        # Format and write error log to file


    return True #TEMP


def insert_args(num_args, event_string, *args):
    # TODO:
        # Find the format codes in the message
        # Convert each arg to int or float to match its code
        # Change %p to %x (Python)
        # Handle %%
        # Print error if arg count != num_args
    return True #TEMP


def main():
    # Create argument parser
    parser = argparse.ArgumentParser(description='Decode error log file using def files.')
   
    # Add arguments for .bin file (mandatory) and def files (optional)
    parser.add_argument(
        'logs_bin',
        type=Path,
        help='Path to the binary log file'
    )
    parser.add_argument(
        '--events_def',
        type=Path,
        default=DEF_DIR / "log_events.def",
        help='Path to the event def file'
    )
    parser.add_argument(
        '--files_def',
        type=Path,
        default=DEF_DIR / "log_files.def",
        help='Path to the filepath def file'
    )
   
    # Read the command line and return object holding all filepath values
    args = parser.parse_args()


    # Ensure all files exist
    for label, path in (("log .bin file", args.logs_bin),
                        ("events def file", args.events_def),
                        ("files def file", args.files_def)):
        if not path.is_file():
            parser.error(f"{label} not found: {path}")


    # Make dicts from def files info
    event_dict = get_event_dict(args)
    file_dict = get_file_dict(args)
   


    # Read the .bin file
    read_bin_file(args, event_dict, file_dict)




# Run script
if __name__ == '__main__':
    main()