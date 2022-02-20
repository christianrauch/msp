#!/bin/sh
find . -regex '.*\.\(cpp\|hpp\)' -exec clang-format-12 -style=file -i {} \;
