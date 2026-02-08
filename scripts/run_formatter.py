Import('env')

import os
import platform
import urllib.request
import stat
import certifi
import subprocess
import glob


flags = env.ParseFlags()  # {'CCFLAGS': [...], 'LINKFLAGS': [...]}
print(flags)

# set configuration
CLANG_FORMAT_VERSION = "21"
DOWNLOAD_DIR = Dir('#/scripts').abspath
SOURCE_DIR  = Dir('#').abspath

BASE_URL = "https://github.com/cpp-linter/clang-tools-static-binaries/releases/download/master-6e612956"

OS_MAP = {
    'Windows': 'clang-format-{ver}_windows-amd64.exe',
    'Linux': 'clang-format-{ver}_linux-amd64',
    'Darwin': 'clang-format-{ver}_macosx-amd64',
}

def is_apple_silicon():
    # Check the machine hardware name directly
    if platform.machine() == 'arm64':
        return True
    
    # Double check if running rossetta
    try:
        command = ["sysctl", "-n", "machdep.cpu.brand_string"]
        brand_string = subprocess.check_output(command).decode().strip()
        if "Apple" in brand_string:
            return True
    except Exception:
        pass
    return False

# check for install 
def get_clang_format():
    
    # get OS
    system = platform.system()
    if system not in OS_MAP:
        print(f"Unsupported OS: {system}")
        Exit(1)

    # if mac and silicon then use bash to install locally and use system clang-format
    if system == "Darwin" and is_apple_silicon():
        print("Checking Clang-Format is installed")

        # check if brew installed
        if env.Execute("brew --version"):
            print("Homebrew is not installed.\nCan not proceed with clang-format download")
            Exit(1)

        elif env.Execute("clang-format --version"): # not installed
            print("Installing Clang-Format")
            status = env.Execute("brew install clang-format")
            print("Installed with code:", status)

            if status: # failed
                Exit(1)
        
        else: # update clang-format to newest version TODO: change to a specific version
            env.Execute("brew upgrade clang-format")
        
        return "clang-format"

        
    filename = OS_MAP[system].format(ver=CLANG_FORMAT_VERSION)
    url = f"{BASE_URL}/{filename}"


    # check if exist
    file_path = f"{DOWNLOAD_DIR}/{filename}"
    if not os.path.exists(file_path):
        print("Downloading Clang-Format {ver}")
        try: 

            os.environ["SSL_CERT_FILE"] = certifi.where()
            print(f"Downloading: {url}")
            urllib.request.urlretrieve(url, filename=file_path)

            # double check if the file path now exists
            print(os.path.exists(file_path), "")

            # if not windows make executable
            if system != 'Windows':    
                st = os.stat(file_path)
                os.chmod(file_path, st.st_mode | stat.S_IEXEC)

            if is_apple_silicon():
                # renaming /usr/local/opt/zstd/lib/libzstd.1.dylib to match the correct path for more cases
                try:
                    brew_prefix = subprocess.check_output(["brew", "--prefix", "zstd"]).decode().strip()
                except subprocess.CalledProcessError:
                    # Fallback/Error if brew is not installed
                    print("Error: brew not found. Please install zstd first.")
                    Exit(1)
                print("change name")
                lib_path = f"{brew_prefix}/lib/libzstd.1.dylib"
                env.Execute(f'install_name_tool -change /usr/local/opt/zstd/lib/libzstd.1.dylib "{lib_path}" {file_path}')
                print("change name exit")
          

        except Exception as e:
            print("Download/Install of Clang-Format failed:", e)
            Exit(1)


    return file_path

def run_formatter(source, target, env):
    clang_format = get_clang_format()
    if not clang_format:
        print("clang-format not found or failed to install.")
        return

    # collect the .c and .h files from our source directories (ignore auto-gen and 3rd party stuff)
    dirs = ["src/application", "src/drivers", "src/common"]
    files = []
    for d in dirs:
        files += glob.glob(os.path.join(SOURCE_DIR, d, "*", "*.c"))
        files += glob.glob(os.path.join(SOURCE_DIR, d, "*", "*.h"))

    if not files:
        print("No source files found to format.")
        return

    # build command with quoted paths. This handles path formatting for mac/linux/windows
    style_file = os.path.join(SOURCE_DIR, "src/third_party/rocketlib/.clang-format")
    files_str = " ".join(f'"{os.path.normpath(f)}"' for f in files)
    cmd = f'"{clang_format}" -i --style="file:{style_file}" {files_str}'

    env.Execute(cmd)


env.AddCustomTarget(
    "Format",
    None,
    run_formatter,  # or any shell/callback
    title="Format",
    description="Run Clang-Format"
)

env.AddCustomTarget(
    "Build and Format",
    None,
    f"pio run",  # or any shell/callback
    title="Build and Format",
    description="Run Clang-Format and build"
)

env.AddPreAction("Build and Format", run_formatter)
