#!/bin/sh
find . -regex '.*\.\(cpp\|hpp\)' -exec clang-format-18 -style=file -i {} \;
