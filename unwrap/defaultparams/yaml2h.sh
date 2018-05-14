#!/bin/sh

dir=`dirname "$0"`
for f in "$dir"/*.yaml; do
echo "// file automatically generated by yaml2h.sh

static const char* DEFAULT_UNWRAP_PARAMS = R\"($(cat "$f"))\";
" > $(echo $f | sed 's/.yaml$/.h/')
done
