### Installation and execution of the code works most seamlessly with VSCode and docker.

1. Install Docker:
    https://docs.docker.com/engine/install/ubuntu/
2. Add user to docker group:
    https://docs.docker.com/engine/install/linux-postinstall/
3. Install VSCode:
    https://code.visualstudio.com/download
4. Install Dev Container extension:
    https://code.visualstudio.com/docs/devcontainers/tutorial


### Get code running:

1. Clone the repository
2. cd into base directory:
```
cd ./drone_master_thesis
```
3. open VSCode:
```
code .
```
4. Press 'Reopen in Container' when VSCode prompts you to do so

5. Open integrated Terminal in VSCode

6. Execute first_run.sh script
```
./first_run.sh
```

7. Close and re-open integrated Terminal

### Code execution:

1. Start simulator with MAVROS in Terminal:
```
roslaunch px4 mavros_posix_sitl.launch 
```

2. Open new integrated Terminal and cd into offboard control directory:
```
cd ./ros/src/offboard_py/scripts
```

3. Start offboard control script:
```
python3 offboard_ctrl.py
```

4. Drone should arm after a few seconds in simulator
5. Publish setopints under 'input/poseSetpoint' topic