#!/bin/bash

source "$(dirname "$0")/_project_env.sh"

if [ -n "$SUDO_USER" ] || [ -n "$SUDO_UID" ]; then
    echo "This script was executed with sudo."
    echo "Use './ops/autorun.sh' instead of 'sudo ./ops/autorun.sh'"
    echo "Exiting..."
    exit 1
fi

# Define the primary ROS runtime cron job
cron_job1="@reboot WITH_CV=true /bin/bash ~/ugv_rpi/ops/start_ros_motion_stack.sh >> ~/ros_motion_autorun.log 2>&1"

# Historical legacy runtime cron jobs kept here only so we can remove them if present
legacy_cron_job_old="@reboot XDG_RUNTIME_DIR=/run/user/$(id -u) ~/ugv_rpi/ugv-env/bin/python ~/ugv_rpi/app.py >> ~/ugv.log 2>&1"
legacy_cron_job_new="@reboot XDG_RUNTIME_DIR=/run/user/$(id -u) PROJECT_DIR=~/ugv_rpi PYTHONPATH=~/ugv_rpi ~/ugv_rpi/ugv-env/bin/python -m legacy_runtime.app_main >> ~/ugv.log 2>&1"

# Define the second cron job for starting Jupyter
cron_job2="@reboot /bin/bash ~/ugv_rpi/ops/start_jupyter.sh >> ~/jupyter_log.log 2>&1"

echo "Configuring autorun for the ROS motion runtime with optional CV."

# Remove any old legacy app autorun if it is still present
if crontab -l 2>/dev/null | grep -Fq "$legacy_cron_job_old"; then
    crontab -l 2>/dev/null | grep -Fv "$legacy_cron_job_old" | crontab -
    echo "Removed historical legacy app.py autorun cron job."
fi
if crontab -l 2>/dev/null | grep -Fq "$legacy_cron_job_new"; then
    crontab -l 2>/dev/null | grep -Fv "$legacy_cron_job_new" | crontab -
    echo "Removed historical legacy legacy_runtime autorun cron job."
fi

# Check if the primary ROS cron job already exists in the user's crontab
if crontab -l | grep -q "$cron_job1"; then
    echo "First cron job is already set, no changes made."
else
    # Add the primary ROS cron job for the user
    (crontab -l 2>/dev/null; echo "$cron_job1") | crontab -
    echo "First cron job added successfully."
fi

# Check if the second cron job already exists in the user's crontab
if crontab -l | grep -q "$cron_job2"; then
    echo "Second cron job is already set, no changes made."
else
    # Add the second cron job for the user
    (crontab -l 2>/dev/null; echo "$cron_job2") | crontab -
    echo "Second cron job added successfully."
fi

CONFIG_FILE=/home/$(logname)/.jupyter/jupyter_notebook_config.py
mkdir -p "$(dirname "$CONFIG_FILE")"
if [ ! -f "$CONFIG_FILE" ]; then
    source "${PROJECT_DIR}/ugv-env/bin/activate" && jupyter notebook --generate-config --allow-root >/dev/null 2>&1 || true
fi
if [ -f "$CONFIG_FILE" ]; then
    grep -Fq "c.NotebookApp.token = ''" "$CONFIG_FILE" || echo "c.NotebookApp.token = ''" >> "$CONFIG_FILE"
    grep -Fq "c.NotebookApp.password = ''" "$CONFIG_FILE" || echo "c.NotebookApp.password = ''" >> "$CONFIG_FILE"
    echo "JupyterLab: password/token = ''."
else
    echo "run jupyter notebook --generate-config failed."
fi

echo "Now you can use the command below to reboot."

echo "sudo reboot"
