#!/bin/sh
cd `dirname $0`
#exec poetry run python -m src.main $@
exec python3 -m src.main $@
