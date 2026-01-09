# DuckieBot Examples

Here we have examples of the following functionalitiy for the DuckieBot:
1) A ROS workspace (src) with blinker package inside. The functionality of this is that is makes the DuckieBot LEDs blink in some colored order. This works by publishing to the /led_pattern topic for the respective vehicle.
2) Same as above but the package is image_saver whose functionality is that it saves an image from the camera every 30 calls. This works by subscribing to the /image/compressed topic for the respective vehicle.


This is the structure of each workspace (shown specifically with the blinker package described in 1) but something similar holds for all the rest).
```
src                   ← ROS workspace
└── blinker/              ← ROS package
    ├── blinker/          ← Python module 
    │   └── blinker.py    ← Your actual ROS node code
    ├── resource/
    │   └── blinker       
    ├── package.xml       ← ROS metadata (we add dependencies)
    ├── setup.py          ← Python packaging + ROS entry points (manually add line for each node)
    └── setup.cfg        
```

### You can clone this repo and run things to see how they work. You cannot push any changes. 

Here are the steps to do to clone:
- Open terminal
- Navigate to where you want to put your cloned repo
- (hopefully you have an SSH key from in class activity, if not follow instruction on Google)
- From Github web, click green code button and copy SSH link
- In terminal, do ``` git clone [INSERT LINK] ```

### Here are the steps to run things:
1. Zip your project:
    1. Window: use GUI
    2. Mac/Linux: run the following line in the terminal in the root of the project:
        ```shell
        zip -r project.zip ./
        ```
2. Copy the project to a server:
   ```shell
   scp project.zip user@rpi-server.local:~
   ```
3. Delete the zip file on your local machine
4. ssh to the server with ```ssh user@rpi-server.local``` and the password is `quackquack`
5. On the server:
    1. Create a directory for your project `mkdir -p project`
    2. Unzip the archive `unzip -d project project.zip` (replace all files if you're asked)
    3. Run ros2 docker container ```docker run -it --network=host -v ./project:/workspace ros:humble```
    4. In the container:
        1. Go to workspace directory: `cd /workspace`
        2. Only the first time,
            3. source ROS `source /opt/ros/humble/setup.bash`
        4. Build the project: `colcon build`
        5. Create a new shell: `bash`
        6. Source the environment: `source ./install/setup.bash`
        7. Set environment variables (according to your duckie)
            1. `export VEHICLE_NAME=duckie01`
            2.  `export USER_NAME=anyname`
        8. To run:
            1. With non-MacOS or without joystick on MacOS, use this format: ```ros2 run package node``` or ```ros2 launch launchPackage launch.xml```
            2. With joystick on MacOS, use the MacOS joystick guide we have provided
          

        
          
*Do see cheat sheet as well for more commands
