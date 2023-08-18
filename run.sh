#!/bin/sh
cd `dirname $0`
venv_name="venv"

# Check if the virtual environment exists
if [ ! -d "$venv_name" ]; then
    echo "Creating virtual environment for visual odometry..."
    python3 -m venv "$venv_name"
fi

# Activate the virtual environment
source "$venv_name/bin/activate"

# Install requirements
echo "Downloading dependencies for visual odometry..."
pip install -r requirements.txt

exec python3 -m src.main $@
#exec python3.9 -m src.main $@
#exec poetry run python -m src.main $@
