# ---- Work in progress of error log decoding script ----

# Will delete all the comments later, it is a bit cramped right now


import sys # for logging errors :)
import re # for regex
import argparse # for command line args
import struct # for converting binary info to float
from pathlib import Path # for getting file path
from collections import namedtuple # for storing event info from def file

# Constants for file paths
REPO_ROOT = Path(__file__).resolve().parents[1]
DEF_DIR = REPO_ROOT / "src" / "application" / "logger" / "log_dictionary"


# .def file regex formats
COMMENTS = r'^(?:\s*/\*.*?\*/)*\s*'   # To account for /* ... */ comments before an entry
EVENT_PATTERN = re.compile(COMMENTS + r'EVENT\(\s*(\w+)\s*,\s*(0[xX][0-9a-fA-F]+|\d+)\s*,\s*(\d+)\s*,\s*"(.*)"\s*\)')
FILE_PATTERN = re.compile(COMMENTS + r'FILE_ID\(\s*(\w+)\s*,\s*(0[xX][0-9a-fA-F]+|\d+)\s*,\s*"(.*)"\s*\)')
TASK_PATTERN = re.compile(COMMENTS + r'TASK_ID\(\s*(\w+)\s*,\s*(0[xX][0-9a-fA-F]+|\d+)\s*,\s*"(.*)"\s*\)')
# First group for decimal points, second for length modifiers, third for type
PLACEHOLDER = re.compile(r'%(?:%|[-+ #0]*\d*(?:\.\d*)?(?:hh|h|ll|l|L)?(?P<plac_type>[diuoxXcfFeEgG]))')
unsigned_plac = "uxXoc"
signed_plac = "di"
float_plac = "fFgGeE"




# nametuple for event dictionary
Event = namedtuple("Event", ["num_args", "message"])


# Error log size
ERR_LOG_SIZE = 32


# Read event def file, create + return dictionary of all error log strings
def get_event_dict(args):
    # Create dictionary that will store event info
    event_info = {}


    # Loop through event def file and store relevant lines
    with open(args.events_def, 'r', encoding='utf-8') as file:
        for line in file:
            # If line doesn't match event pattern, discard
            m = EVENT_PATTERN.match(line)
                # Won't work if event defintion spread out over multiple lines.
                # TODO: Ask if need to account for this
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

# Read task def file, create + return dictionary of all tasks
def get_task_dict(args):
    # Create dictionary that will store event info
    task_info = {}
   
    # Loop through file def file and store relevant lines
    with open(args.tasks_def, 'r', encoding='utf-8') as file:
        for line in file:
            m = TASK_PATTERN.match(line)
            if m is None:
                continue
            enum_name, task_id, task_name = m.groups()
            task_info[int(task_id, 0)] = task_name


    # TEMP: Check if dict is defined properly
        #for task_id, path in list(task_info.items())[:5]:
            #print(task_id, task_name)        


    return task_info




def read_err_file(args, event_dict, file_dict, task_dict):
    lines = []
    with open(args.logs_err, "rb") as file:
        # lambda is mini function, repeats reading 32 bytes at a time until end of file
        for record in iter(lambda: file.read(ERR_LOG_SIZE), b""):
            # Print to console if partial record encountered
            if len(record) < ERR_LOG_SIZE:
                print(f"INFO: partial record ({len(record)} bytes) encountered.")
            # Call function that will take binary line and translate to string message
            line = translate_log(record, event_dict, file_dict, task_dict)
            if line is not None:
                lines.append(line)
    # Return list of translated lines
    return lines


def translate_log(chunk, event_dict, file_dict, task_dict):
    timestamp = int.from_bytes(chunk[0:4], byteorder="little") # int
    event_id = int.from_bytes(chunk[4:6], byteorder="little") # key

    event_tuple = event_dict.get(event_id, "unknown") # named tuple
    if (event_tuple == "unknown" and event_id != 0):
            print(f"Unknown event ID: {event_id}", file=sys.stderr)
            return "Unknown event id, no message found"
    num_args = event_tuple.num_args # int
    message = event_tuple.message # string

    task_id = int.from_bytes(chunk[6:7], byteorder="little") # key
    task_name = task_dict.get(task_id, "unknown")
    # If task id not found in dict, log unknown id
    if (task_name == "unknown" and task_id != 0):
        print(f"Unknown task ID: {task_id}", file=sys.stderr)

    file_id = int.from_bytes(chunk[7:8], byteorder="little") # key
    filepath = file_dict.get(file_id, "unknown") # string
    if (filepath == "unknown" and file_id != 0):
        print(f"Unknown file ID: {file_id}", file=sys.stderr)

    if (task_id == 0):
        task_name = "LOG_TASK_ERROR" # TODO: Ask is this the right task name?
    if (file_id == 0):
            return None
    if (event_id == 0):
            return None
    
    line_num = int.from_bytes(chunk[8:10], byteorder="little") # int
    counter = int.from_bytes(chunk[10:12], byteorder="little") # int
    # args = int.from_bytes(chunk[12:32], byteorder="little")
    binary_args = [chunk[12:16], chunk[16:20], chunk[20:24], chunk[24:28], chunk[28:32]]

    # Insert args into message with arg placeholders
    msg_with_args = insert_args(message, binary_args)

    # Timestamp: Event message occured X times, TASK: task, FILE: filepath, LINE: line
    formatted_msg = f"{timestamp}: {msg_with_args} occured {counter} times, TASK: {task_name}, FILE: {filepath}, LINE: {line_num}."
    

    return formatted_msg


def insert_args(message, binary_args):
    # Currently does not accomodate for doubles, etc.
    decoded_args = []
    i = 0
    matches = PLACEHOLDER.finditer(message)
    for m in matches:
        arg_type = m.group('plac_type')
        if (arg_type is None): continue
        if (arg_type in unsigned_plac):
            decoded_args.append(int.from_bytes(binary_args[i], byteorder="little"))
        elif (arg_type in signed_plac):
            decoded_args.append(int.from_bytes(binary_args[i], byteorder="little", signed=True))
        elif (arg_type in float_plac): # This can also just be else, I kept it elif for readability
            # '<f' means little-endian, [0] to pull float out of the tuple
            decoded_args.append(struct.unpack('<f', binary_args[i])[0])
        i += 1
        
    msg_with_args = message % tuple(decoded_args)

    # TODO: Account for escape sequences like \n, \t IF NEEDED
    
    # TEMP for debugging
    #print(msg_with_args)

    return msg_with_args


def main():
    # Create argument parser
    parser = argparse.ArgumentParser(description='Decode error log file using def files.')
   
    # Add arguments for .err file (mandatory) and def files (optional)
    parser.add_argument(
        'logs_err',
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
    parser.add_argument(
        '--tasks_def',
        type=Path,
        default=DEF_DIR / "log_tasks.def",
        help='Path to the task def file'
    )
   
    # Read the command line and return object holding all filepath values
    args = parser.parse_args()


    # Ensure all files exist
    for label, path in (("log .err file", args.logs_err),
                        ("events def file", args.events_def),
                        ("files def file", args.files_def),
                        ("tasks def file", args.tasks_def)):
        if not path.is_file():
            parser.error(f"{label} not found: {path}")


    # Make dicts from def files info
    event_dict = get_event_dict(args)
    file_dict = get_file_dict(args)
    task_dict = get_task_dict(args)
   


    # Read the .err file, put into array of error log lines
    lines = read_err_file(args, event_dict, file_dict, task_dict)

    # Write lines into .txt file
    with open('decoded_error_logs.txt', 'w', encoding='utf-8') as f:
        for line in lines:
            f.write(line + "\n")


# Run script
if __name__ == '__main__':
    main()