#!/bin/sh

${CLANG_FORMAT:-clang-format} -i --style=file:src/third_party/rocketlib/.clang-format src/application/*/*.c src/application/*/*.h src/application/*/*/*.c src/application/*/*/*.h src/drivers/*/*.c src/drivers/*/*.h src/common/math/*.c src/common/math/*.h
