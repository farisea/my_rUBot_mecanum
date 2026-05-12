# Project for Traffic Signal Detection with YOLO

We will describe the Computer Vision based method to identify the Traffic Sign.

Training models: 
- https://roboflow.com/
- https://github.com/ultralytics/ultralytics
- https://docs.ultralytics.com/es/usage/python/#how-do-i-train-a-custom-yolo-model-using-my-dataset

For this project we have created a new package "my_robot_ai_identification" where we have used 2 strategies to perform signal identification:
- Keras with tensorflow
- YOLO 

## **1. ROS2 packages installation**

The needed Installation for YOLO identification is only to install "ultralytics" on the ROS2 Humble environment. Open a terminal and type:
````shell
pip install ultralytics
pip3 uninstall numpy
pip3 install "numpy<2.0"
````

For simulation, you won't be able to use TheConstruct environment. You have to use your **Docker container ROS2 custom environment**:
- Use VScode and clone your project repository
- Edit `docker-compose.yaml` from `network_config/humble`
- Comment or delete the environment variables: 
    - ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    - ROS_STATIC_PEERS=192.168.1.54
    - CYCLONEDDS_URI=file:///config/cyclonedds_pc.xml
    - Choose DISPLAY:
        - DISPLAY=${DISPLAY} #Ubuntu
        - DISPLAY=host.docker.internal:0.0 #Windows 
- Verify in PC-win `entrypoint_pc.sh` has `LF` NOT `CRLF`
- Open a terminal in `network_config/humble` and write:
    ````bash
    docker compose up
    ````
- Install for graphical interface:
    - For Ubuntu: https://mac.getutm.app/
    - For Windows: https://sourceforge.net/projects/vcxsrv/files/latest/download
- Open a VScode window attached to the created container
- Clone your project repository
- open `.bashrc` file and add:
    ````xml
    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    source /root/my_rUBot_mecanum/install/setup.bash
    export GAZEBO_MODEL_PATH=/root/my_rUBot_mecanum/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
    cd /root/my_rUBot_mecanum
    ````
You have now your custom docker ROS2 workspace ready!


## **2. Robot Navigation**

To proceed with the signal identification we first bringup the robot and navigate from initial pose to final target.

- Bringup the robot
    - In simulation:
    ````shell
    ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml use_sim_time:=true x0:=0.5 y0:=-1.5 yaw0:=1.57 robot:=rubot/rubot_mecanum.urdf custom_world:=square4m_sign.world
    ````
    >Important: Include a traffic signal in the world. When using "square4m_sign.world" you can change the sign model on line 30 changing the traffic sign model name
    - In real robot rUBot or LIMO the bringup is already made when turned on

- Generate a map
    - In simulation:
        ````shell
        ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=true
        ````
    - In real robot (rUBot or LIMO):
        ````shell
        ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=false
        ````
- Save the map in my_robot_navigation2/map folder with:
    ````shell
    cd src/Navigation_Projects/my_robot_navigation2/map/
    ros2 run nav2_map_server map_saver_cli -f map_square4m_sign
    ````
- Navigate using the Map:
    - In simulation:
        ````bash
        ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=true map_file:=map_square4m_sign.yaml params_file:=rubot_sw.yaml
        ````
        >In case we want to priorize the lidar data from odometry data we will use `rubot_sw_lidar.yaml`. Equivalent names are found for rUBot real robot.

        ![](./Images/07_Yolo/10_nav_sw.png)

    - In real robot rUBot:
        ````shell
        ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=map_project.yaml params_file:=rubot_real_lidar.yaml
        ````
        > Be sure the Odometry message is zero when starting the navigation.

        > In the case of `Limo real robot`:
        >- Because the bringup is done without the LIMO robot model. The only frames available are
        >    - odom: as a ``base_frame_id``
        >    - base_link: as the ``robot_base_frame``
        >- We have to create "LIMO_real.yaml" file in "param" folder correcting base_frame_id: "odom" (instead of base_footprint)
        

## **3. Signal detection**

First we want to test the classification model prediction:
- We have created a new node `rubot_rt_yolo_cls.py` that:
    - Subscribes to a ROS2 camera image topic.
    - Loads a YOLO classification model.
    - Runs real-time image classification.
    - Detects the most probable traffic sign class.
    - Publishes classification results in a custom ROS2 message.
    - Publishes an annotated image with prediction labels.
    - Uses configurable ROS2 parameters for model, topic, confidence, and image size.
    ````bash
    ros2 run my_robot_ai_identification rubot_rt_yolo_cls_exec \
    --ros-args \
    -p modelYolo:=best_cls.pt
    ````
Once the identification is validated, we have created a new node `rubot_identification_yolo_cls.py` that:
- Subscribes to a camera image topic.
- Loads a YOLO classification model.
- Classifies each camera image to detect a traffic sign.
- Publishes the prediction result to /Yolov8_Inference.
- Publishes an annotated image to /inference_result.
- Loads known traffic sign positions from a YAML file.
- Reads the robot pose from TF: map -> base_link.
- Checks if the robot is close enough to the detected sign.
- Applies simple traffic-sign logic: STOP, Ceda, Prohibido, Derecha, Izquierda.
- Creates a navigation waypoint near the detected sign.
- Publishes the waypoint to /traffic_waypoint.
- Uses cooldown and hold times to avoid repeated reactions.

In `config` folder we have to review the following files for the correct parameters:
- `sign_positions_real.yaml` 
- `yolo_params_real.yaml` 

The schematic nodes, topics and messages are shown below:
    ![](./Images/07_Yolo/09_yolo_detection_topics.png)


**Hardware** Test in real robot:
- If you want to execute on real `rUBot robot`, you have to execute:
    ````shell
    ros2 launch my_robot_ai_identification rubot_identification_yolo.launch.py
    ````
    > Review the correct parameters!


## **4. Custom Navigation with signal detection**

Considering the previous `object_detection` node, we have created a new `custom_nav2` node that:
- Creates a ROS2 navigation task using Nav2.
- Reads `yolo_targets_real.yaml` the initial pose, traffic sign waypoint and final target pose.
- Uses BasicNavigator from Nav2 Simple Commander.
- Sets the robot initial pose in the map.
- Waits until Nav2 becomes active.
- Navigates the robot to the predefined signal waypoint.
- Subscribes to /traffic_waypoint.
- Waits for a waypoint generated by the YOLO traffic-sign node.
- Navigates to the received traffic waypoint if available.
- Continues to the final target pose.


In `config` folder we have to review the following files for the correct parameters:
- `yolo_targets_real.yaml`

The schematic nodes, topics and messages are shown below:
    ![](./Images/07_Yolo/12_yolo_custom_nav2.png)

To launch the robot Custom Navigation with signal detection, use:
- Launch the `Navigation2 stack`, use:
    ````shell
    ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=map_project.yaml.yaml params_file:=rubot_real_lidar.yaml
    ````
- Launch the `object_detection` node with:
    ````shell
    ros2 launch my_robot_ai_identification rubot_identification_yolo.launch.py 
    ````
    > Review the parameters!
- Launch the `custom_nav2` node with:
    ````shell
    ros2 launch my_robot_ai_identification rubot_targets_yolo.launch.py
    ````
    > Review the parameters!

We have created a full launch file `ai_navigation.launch.py` that launches the 3 nodes together:
- `Navigation2 stack`
- `object_detection` node
- and starts after a controlled delay, the `custom_nav2` node that integrates the detected waypoint in the navigation2 stack

You can launch all nodes with only one launch file:
````bash
ros2 launch my_robot_ai_identification ai_navigation.launch.py \
  map_file:=my_map2.yaml \
  params_file:=rubot_real_lidar.yaml \
  yolo_params:=yolo_params_real.yaml \
  nav_params:=yolo_targets_real.yaml \
  signs_file:=sign_positions_real.yaml \
  use_sim_time:=false
````


| AI Identification and Navigation video | Code execution video |
|----------|------------|
| [▶️ rUBot Traffic signal Detection & Autonomous Navigation](./Images/07_Yolo/Yolo_left.mp4) | [▶️ rUBot code execution](./Images/07_Yolo/YoloSignalWaypoint.webm) |