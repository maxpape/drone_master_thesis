#!/bin/bash

rsync -a --progress -v /drone_master_thesis/px4_changes/ /drone_master_thesis/PX4-Autopilot/ --exclude=copy_changes.sh