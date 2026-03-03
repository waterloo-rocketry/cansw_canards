#!/bin/sh
set -x
${CLANG_TIDY:-clang-tidy} --warnings-as-errors="*" --checks="clang-*,misc-*" --extra-arg-before="-std=c99" --extra-arg-before="-pedantic" --extra-arg-before="-Isrc" --extra-arg-before="-Isrc/third_party/cubemx_autogen/Middlewares/Third_Party/FreeRTOS/Source/include" --extra-arg-before="-Isrc/third_party/canlib" src/application/*/*.c src/application/*/*.h src/application/*/*/*.c src/application/*/*/*.h  src/common/math/*.c src/common/math/*.h
