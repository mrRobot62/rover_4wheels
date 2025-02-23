#!/bin/bash
#File: tree2md
#Description: Convert output of tree utility to Markdown in a pretty format

tree=$(tree -f --noreport --charset ascii $1 |
sed -e 's/| \+/  /g' -e 's/[|`]-\+/ */g' -e 's:\(* \)\(\(.*/\)\([\*^/\*]\+\)\):\1[\*\4\*](\2):g' -e 's/^\( \* \)/├──/gm' -e 's/^\(   \* \)/       └──/gm' -e 's/^\(     \* \)/       └──/gm')
printf "# Code Struct:\n\n${tree}\n\n"