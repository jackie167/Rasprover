#!/bin/bash

if [ -n "$SUDO_USER" ] || [ -n "$SUDO_UID" ]; then
    echo "This script was executed with sudo."
    echo "Use './autorun.sh' instead of 'sudo ./autorun.sh'"
    echo "Exiting..."
    exit 1
fi

# Define the primary ROS runtime cron job
cron_job1="@reboot /bin/bash ~/ugv_rpi/start_ros_full_stack.sh >> ~/ros_full_autorun.log 2>&1"

# Legacy app runtime cron job kept here only so we can remove it if present
legacy_cron_job="@reboot XDG_RUNTIME_DIR=/run/user/$(id -u) ~/ugv_rpi/ugv-env/bin/python ~/ugv_rpi/app.py >> ~/ugv.log 2>&1"

# Define the second cron job for starting Jupyter
cron_job2="@reboot /bin/bash ~/ugv_rpi/start_jupyter.sh >> ~/jupyter_log.log 2>&1"

echo "Configuring autorun for the ROS full stack runtime."

# Remove the old legacy app autorun if it is still present
if crontab -l 2>/dev/null | grep -Fq "$legacy_cron_job"; then
    crontab -l 2>/dev/null | grep -Fv "$legacy_cron_job" | crontab -
    echo "Removed legacy app.py autorun cron job."
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
    source "$PWD/ugv-env/bin/activate" && jupyter notebook --generate-config --allow-root >/dev/null 2>&1 || true
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
