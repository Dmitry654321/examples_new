# DuckieBot Examples

Here we have examples of the following functionality for the DuckieBot:
1) A ROS workspace (src) with blinker package inside. The functionality of this is that is makes the DuckieBot LEDs blink in some colored order. This works by publishing to the /led_pattern topic for the respective vehicle.
2) Same as above but the package is image_saver whose functionality is that it saves an image from the camera every 30 calls. This works by subscribing to the /image/compressed topic for the respective vehicle.
3) We also have a joystick example whose functionality is to drive the robot. You can control the robot use 'wasd' and 'space' to stop. This also demonstrates the launch file appraoch instead of an individual node for those who want to look into that.


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

  
### How to run things:
1. #### Local Server
    1. You'll want to save your changes to git
        1. ```git add .``` to stage all files
        2. ```git commit -m "[INFORMATIVE MESSAGE]"``` to explain what you are sending
        3. ```git push``` to finally send everything
    2. Connect to remote server via ssh
       ```bash
        ssh user@rpi-server.local
       ```
       with the password ```quackquack```
2. #### Remote Server
    1. You'll want to get your changes from git
        1. ```git pull``` to get latest changes on main branch
    2. Run ```make build```
    3. Run ```make run```
3. #### Docker Container Within Remote Server
    1. Launch the necessary ros2 packages using
       ```bash
       ros2 run package node
       ```
       ```bash
       ros2 launch launchPackage launchFile
       ```
   
