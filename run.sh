#!/bin/sh
cd `dirname $0`

# Be sure to use `exec` so that termination signals reach the python process,
# or handle forwarding termination signals manually
# echo "Removing old debug files."
# rm -r "results"/*
exec python3.9 -m src.main $@
