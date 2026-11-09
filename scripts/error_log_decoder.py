# ---- Work in progress of error log decoding script ----


# Find where log file is and where def files are
import argparse
from pathlib import Path
# Constants for file paths
REPO_ROOT = Path(__file__).resolve().parents[1]
DEF_DIR = REPO_ROOT / "log_dictionary"


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




# Read def files and create a dictionary of error codes and their descriptions
    # Loop through file
    # Ignore comments and blank lines
    # Convert hex ID into a base-10 number


# Checks
    # Check that ID appears twice in the def files
    # Check that each event has max 5 arguments


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
