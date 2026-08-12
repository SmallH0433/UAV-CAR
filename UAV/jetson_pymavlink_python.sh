#!/usr/bin/env bash
set -euo pipefail

# Jetson Nano's preinstalled NumPy uses an instruction set that can crash on
# import. Keep the existing AI stack untouched and run pure-Python MAVLink
# tools without loading /usr/local NumPy.
export PYTHONPATH="/home/jetson/.local/lib/python3.6/site-packages:/usr/lib/python3/dist-packages:/usr/lib/python3.6/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -S "$@"
