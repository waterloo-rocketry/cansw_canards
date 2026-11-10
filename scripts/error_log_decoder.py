# ---- Work in progress of error log decoding script ----


# Find where log file is and where def files are
import argparse
from pathlib import Path
# Constants for file paths
REPO_ROOT = Path(__file__).resolve().parents[1]
DEF_DIR = REPO_ROOT / "log_dictionary"

def remove_comments_and_whitespace(lines):
    cleaned_lines = []
    for line in lines:
        # Remove whitespace
        line = line.strip()
        # Ignore full-line comments and blank lines
        if line and not line.startswith('//') and not line.startswith('/*') and not line.startswith('*') and not line.startswith('*/'):
            cleaned_lines.append(line)
        # TODO: Ignore comments within lines, allow for code lines to start with *,
        # ignore multi-line comments that don't start with *,
        # allow for code lines that start with /* comment */ and then have code written after
        # Better idea: Just take string from EVENT to ")
    return cleaned_lines

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
    filepaths = parser.parse_args()

    # filepaths.logs_bin is the filepath to the bin file
    # etc.


# Read def files and create a dictionary of error codes and their descriptions
    # Loop through event def file and store relevant lines
    with open(filepaths.events_def, 'r') as file:
        # Store all lines in a list
        event_lines = file.readlines()
        # Call function that removes comments and whitespace
    event_lines = remove_comments_and_whitespace(event_lines)
    # TEMP: Print first 30 cleaned lines to check that function works
    #for line in event_lines[:30]:
        #print(line) # TEMP: Currently fine, still need to refine cleaning function though 

    # Create dictionary that will store event info
    event_info = {}

    # In case event id is invalid
    dummy_event_id = -1

    # Store all event info in dictionary (based on id number)
    for line in event_lines:
        # Split line based on comma delimiter into:
            # EVENT(ENUM_NAME
            # ID
            # NUM ARGS
            # "STRING")
        # These are all stripped of whitespace and stored in the items list
        items = [item.strip() for item in line.split(",", 3)]
        # Maxsplit 3 since at most 3 commas (starting from the left) before string occurs
        # (doesn't split string itself when encountering strings with commas)
            
        # Check if right number of items
        if len(items) != 4:
            print("Event def file entry does not follow expected format.")
            # TODO: Figure out what to do if this occurs
            # Write event info to invalid file, then remove from event_lines, then continue?


        items.pop(0) # Get rid of EVENT(ENUM_NAME
        # TODO: Ask if we need to keep the enum name too
        # (in case we want to search the program files after with the enum name)
        items[-1] = ((items[-1].removesuffix(")")).removesuffix("\"")).removeprefix("\"")
        # TEMP: Check if split line properly
        #print(items)

        # Convert ID string to ID int
        try: 
            # 0 argument makes it able to handle ids of any base
            # (just in case id was written as hexadecimal)
            event_id = int(items[0], 0)
            # TODO: Check if event_id already in dict.
            # If so, write event info to invalid file, then remove from event_lines, then continue?
        except ValueError:
            print("Invalid input: ID ",  items[0], " was not an integer")
            # TODO: Figure out what to make items[0] if ValueError occurs
            # Write event info to invalid file, then remove from event_lines, then continue?
            # Below is a temporary fix with dummy_event_id
            event_id = dummy_event_id # Set to dummy int if case id conversion fails
            dummy_event_id -= 1
        items.pop(0)

        # Convert # of args string to # of args int
        try:
            items[0] = int(items[0])
        except ValueError:
            print("Invalid # of args: Could not convert", items[0], " to int.")
            # TODO: Figure out what to make items[0] if ValueError occurs
            # Write event info to invalid file, then remove from event_lines, then continue?
            
        if items[0] < 0 or items[0] > 5:
            print("Invalid # of args (", items[0], ") associated with event id ", event_id)
            # TODO: Figure out what to do if this occurs
            # Write event info to invalid file, then remove from event_lines, then continue?

        # Store all event info at that id number
        event_info[event_id] = items

        # TEMP: Check if dict is defined properly
        #print(event_info)


# Read the files def and make a dict for that too


# Read the log file


# Split into chunks of 32 bytes (loop in steps of 32)


# Turn each set of 32 bytes into named numbers (event ID, timestamp, etc.)


# Use the dictionary to convert named numbers into strings


# Plug the numeric arguments into the string


# Format and print the error log


# Write the output to a file (if we want to do this)


# Run script
if __name__ == '__main__':
    main()
