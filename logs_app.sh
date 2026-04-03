#!/bin/bash

LOG_FILE="$HOME/ugv.log"

if [ ! -f "$LOG_FILE" ]; then
    echo "Log file not found: $LOG_FILE"
    exit 1
fi

tail -f "$LOG_FILE"
