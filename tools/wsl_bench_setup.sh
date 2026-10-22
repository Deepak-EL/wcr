#!/usr/bin/env bash
# User-level pip + moderngl/glfw for the graphics benchmark (no sudo).
cd /tmp
python3 -m pip --version >/dev/null 2>&1 || { curl -sSLO https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py --user -q; }
python3 -m pip install --user -q moderngl glfw numpy 2>&1 | tail -2
python3 -c 'import moderngl, glfw; print("wsl ok", moderngl.__version__)'
