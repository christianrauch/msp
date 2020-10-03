#!/bin/sh
find . -regex '.*\.\(cpp\|hpp\)' -exec clang-format-6.0 -style=file -i {} \;

