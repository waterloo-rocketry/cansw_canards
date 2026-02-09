# Canard Board Firmware

Firmware for 2025-26 Canard Board, performing state estimation and control for the 2nd iteration of the canards system. Project documentation can be found in the team Google Drive.
In 2024-25, this project was firmware for Processor Board in the first canards system

## Project Structure

- `src/drivers/`: custom peripheral driver modules
- `src/application/`: high-level application logic modules
- `src/third_party/`: third-party libraries, including CubeMX auto-gen files
- `src/common/`: shared resources specific to this project
- `tests/`: everything for [testing](#Unit-Testing)

## Developer Setup

### 1. Clone repo

- Clone repo and initialize submodules: ` git clone --recurse-submodules https://github.com/waterloo-rocketry/cansw_processor_canards`

### 2. Firmware dev in PlatformIO: edit, build, flash, debug

All firmware dev is done in VSCode using PlatformIO (previously using devcontainer) for easy cross-platform usage.

- Install the PlatformIO extension in VSCode and open this folder
- PlatformIO should auto-detect this project by recognizing the `platformio.ini` file
- Use the PlatformIO tab to Build and Upload. Ex, `Project Tasks > General > Build`
- Use the "Run and Debug" tab to debug on the board once connected via ST-Link
- If needed, switch between debug/release environments: `Ctrl+Shift+P > PlatformIO: Pick Project Environment`
- Auto-format the project code: `Project Tasks > Custom > Format`

### 3. Run unit tests in devcontainer

The devcontainer has everything installed to run unit tests using CMake. One could also install all the dependencies locally if desired but the devcontainer is recommended to avoid "but it works on my machine!" issues.

Devcontainer setup:

- [Install devcontainers](https://code.visualstudio.com/docs/devcontainers/tutorial)
- In a new vscode window, use `Ctrl+Shift+P`, search `Dev Containers: Open Folder In Container...`, then select this project folder
    - The first time opening the project will take several minutes to build the devcontainer. Subsequent times will be instant.

Run tests:

- Open the CMake plugin tab from the sidebar
- Under `Configure`, select which build type you want
- Hover over `Build`, click the build icon to build the configuration
    - The build preset should automatically be selected (eg, `Build Firmware (Debug) preset`)

## Unit Testing

We use GoogleTest and Fake Function Framework (fff) for unit testing. All testing-related files are in `tests/`.

- Tests are built from `tests/CMakeLists.txt` which is separate from the project's main build config. Building and running tests is done via cmake.
- Test source code should be written in `tests/unit/`.
- Mocks should be made with fff in `tests/mocks/`.

### Add a test

- Add a new test group file in `tests/unit/`. See `test_dummy.cpp` for example of test structure.
- Add the test group to the cmake build system by editing `tests/CMakeLists.txt`:
    - At the bottom of the file, add the new test group using the `add_test_group()` helper.
      (Read the comments + existing examples explaining how it works)

### Add a mock

We do not include the STM32 HAL library nor FreeRTOS when compiling the project for unit tests.
So if a source file uses a HAL or FreeRTOS file, those files and their functions must be mocked using fff.
This also lets us mock any module that a test interacts with but doesn't test.

Example 1:

- `src/drivers/gpio/gpio.c` uses FreeRTOS semaphores via `#include "semphr.h`.
    - In the actual firmware, the real `semphr.h` is included when compiling. But for unit tests, the real `semphr.h` is not included when compiling. So, the unit tests fail to compile (it can't find a `semphr.h` file).
        - To correct this we add a "fake" `semphr.h` in `tests/mocks/semphr.h`. All files in this folder are included when compiling unit test, so the tests now compile.
    - The gpio code uses functions from the real `semphr.h` like `xSemaphoreTake()`. These don't exist in our fake `semphr.h` yet.
        - To correct this we need to create a mock `xSemaphoreTake()` function using fff.
          [fff's Readme](https://github.com/meekrosoft/fff?tab=readme-ov-file#hello-fake-world) describes how to create fake functions. Here's the mock for `xSemaphoreTake()`:

            ```
            // The func to mock: BaseType_t xSemaphoreTake(SemaphoreHandle_t arg0, TickType_t arg1)

            DECLARE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreTake, SemaphoreHandle_t, TickType_t);
            ```

            First we put the _declaration_ (`DECLARE_FAKE...`) in `mocks/semphr.h`. Then, put the actual _definition_ (`DEFINE_FAKE...`) in the corresponding `mocks/semphr.c`.

- Now in the gpio tests, we can access the mocked semaphore functions via fff to test that the gpio code uses semaphores correctly.

Example 2:

- `src/application/estimator/estimator.c` takes input from flightphase via `#include "application/flight_phase/flight_phase.h`.
    - To test estimator, we will test various inputs from flightphase. But we don't want to be testing flightphase at the same time.
      So, mock flightphase.
    - The real `flight_phase.h` header is built in unit tests, but the source file `flight_phase.c` is not.
        - Since a function _declaration_ for `get_flight_phase()` exists in the header, we can use fff's `DEFINE_FAKE...` to create a
          fake _definition_ of that `get_flight_phase()`. That definition should be in the test .cpp file, or in the mocks folder if it's applicable for wider use.
        - (Note: unlike example 1 where the header isn't included in unit tests, the flightphase header is included so it doesnt need a `DECLARE_FAKE...`)

### Run/debug tests

- Build in vscode using cmake (see step 3 above)
    - The default `Build Unit Tests With Coverage preset` also automatically runs all tests and generates coverage report.
    - View the coverage report html pages in `build/test/coverage_report` in a local browser
- Use the vscode cmake `Launch` and `Debug` tabs to run/debug individual test groups (cmake shows the available test groups)

## Code Standards

This project follows the [team-wide embedded coding standard](https://docs.waterloorocketry.com/general/standards/embedded-coding-standard.html).

- Use the `Format` target via PlatformIO to format via clang-format (see step 2)

- Rocketlib is included at `src/third_party/rocketlib`.
- Developers should be aware of the BARR Standard and make an effort to follow it.

## Adding Log Messages

When adding a new type of data log message, all of the following should be updated:

- src/application/logger/log.h
    - Add a new enum value to `log_data_type_t`:
        ```diff
         typedef enum {
        +    LOG_TYPE_XXX = M(unique_small_integer),
         } log_data_type_t;
        ```
    - Add a struct definition of your message's data fields to `log_data_container_t`:
        ```diff
         typedef union __attribute__((packed)) {
        +    struct __attribute__((packed)) {
        +        uint32_t l;
        +        float f;
        +        // ...
        +    } typename;
         } log_data_container_t;
        ```
- scripts/logparse.py
    - Add a new format spec to the `FORMATS` dict:
        ```diff
         FORMATS = {
        +    M(unique_small_integer): Spec(name, format, [field, ...]),
         }
        ```
