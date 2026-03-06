#!/bin/sh
set -x
${CLANG_TIDY:-clang-tidy} --warnings-as-errors="*" \
						  --checks="clang-*,misc-*" \
						  --extra-arg-before="-std=c99" \
						  --extra-arg-before="-pedantic" \
						  --extra-arg-before="-Isrc" \
						  --extra-arg-before="-Isrc/third_party/cubemx_autogen/Middlewares/Third_Party/FreeRTOS/Source/include" \
						  --extra-arg-before="-IInc" \
						  --extra-arg-before="-Isrc/third_party/canlib" \
						  --extra-arg-before="-Isrc/third_party" \
						  --extra-arg-before="-Isrc/third_party/CMSIS-DSP/Include" \
						  --extra-arg-before="-Isrc/third_party/cubemx_autogen/Drivers/STM32H7xx_HAL_Driver/Inc" \
						  --extra-arg-before="-Isrc/third_party/cubemx_autogen/Drivers/CMSIS/Include" \
						  --extra-arg-before="-Isrc/third_party/cubemx_autogen/Drivers/CMSIS/Device/ST/STM32H7xx/Include" \
						  --extra-arg-before="-Isrc/third_party/cubemx_autogen/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" \
						  --extra-arg-before="-Isrc/third_party/cubemx_autogen/Middlewares/Third_Party/FatFs/src" \
						  --extra-arg-before="-DSTM32H733xx" \
						  src/application/*/*.c \
						  src/application/*/*.h \
						  src/application/*/*/*.c \
						  src/application/*/*/*.h \
						  src/drivers/*/*.c \
						  src/drivers/*/*.h \
						  src/common/math/*.c \
						  src/common/math/*.h
