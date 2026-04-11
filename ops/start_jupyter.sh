#!/bin/bash

source "$(dirname "$0")/_project_env.sh"
[ -f ~/.bashrc ] && source ~/.bashrc
cd "$PROJECT_DIR" && source ugv-env/bin/activate && jupyter lab --ip=0.0.0.0 --port=8888 --no-browser
