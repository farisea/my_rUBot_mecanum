# Project for Traffic Signal Detection with YOLO

We will describe the Computer Vision based method to identify the Traffic Sign.

Training models: 
- https://roboflow.com/
- https://github.com/ultralytics/ultralytics
- https://docs.ultralytics.com/es/usage/python/#how-do-i-train-a-custom-yolo-model-using-my-dataset

For this project we have created a new package "my_robot_ai_identification" where we have used YOLO strategy to perform signal identification:

The signals that we want to identify are:
- Stop
- Turn Right
- Turn Left
- Give Way
- No Entry 

![](./Images/07_Yolo/TrafficSigns.png)

## **3. Model Training**

To properly train a model we will use "roboflow":
- Open a new google tab: https://roboflow.com/
    ![](./Images/07_Yolo/01_roboflow.png)
- Select "Get Started" or "Sign In" and "Continue with Google"
- Select a Name of the workspace (i.e. TrafficSignals)
- Select "Public Plan"
- You will have a maximum of 4 invites available for your project partners to collaborate in the model generation. We suggest a role of "Admin" for Invite team members
- Create a workspace
- Answer some objective questions
- Select "What type of model would you like to deploy?". Type "Object Detection"
    ![](./Images/07_Yolo/02_Object_detection1.jpg)
- There is a short Roboflow tutorial video: https://blog.roboflow.com/getting-started-with-roboflow/

- Create a project in our created "Workspace":
    - Select Projects and choose ``new project``, choose a name and click on `Continue with Public`
        ![](./Images/07_Yolo/02_Object_detection1.jpg)
    - Select `Use Traditional Model Builder Instead` to have whole control of YOLO model in Robotic projects
    - You have 5 different classes: Stop, Right, Left, Give, Forbidden
    - You will have in your local PC one folder per Class. To take pictures with the robot camera and save this pictures in a local folder, you have to run a custom node, that:
        - subscribes to the `/image_raw` topic
        - takes one image each x seconds
        - save each image in a local folder with the format. `folder_name_001.jpg`, etc
        ````python
        python3 0_capture_topic_images0.py --ros-args \
        -p image_topic:=/image_raw \
        -p output_folder:=handshake \
        -p capture_interval:=2.0
        ````
    -Choose `Select Folder` to upload pictures from a local folder. Upload all the images on this project.
        ![](./Images/07_Yolo/04_Project2.png)
    - Type ``save&continue`` and ``start labeling`` to label all traffic signs pictures
    - You can assign some pictures to different Invited team members
    - Select ``start anotating``. You will do it for each Class.
        ![](./Images/07_Yolo/05_Label.png)
    - If you make an error, type ``layers`` 3point menu and change class
        ![](./Images/07_Yolo/06_Label_error.png)
    - When finished go back (left corner arrow) and select ``add xx images to Dataset``.
    - Select Method ``use existing values`` and press ``add images``
    - Select ``train model`` and ``custom training``
    - Edit ``train test/split`` select ``balance`` (select % of training (80%) / Validating (15%) / Test (5%))
        ![](./Images/07_Yolo/07_train_balance.png)
    - select ``continue`` for the other options
    - select ``augmentation`` and ``shear`` to proper consider rotations in x and y axis
        ![](./Images/07_Yolo/08_shear.png)
    - type ``create``
    - type ``download Data set`` choose format ``yolov8`` and ``Download zip to computer``. Save this zip file to your computer. This contains images (for train, valid and test) and data.yaml used in the next section to obtain the final model.

 <img src="./Images/07_Yolo/09_DataSet.png" width="400"/>  <img src="./Images/07_Yolo/09_DataSet2.png" width="200"/> 


## **4. Signal prediction**

In TheConstruct environment
- `1_train_model.py`: Train the model on pre-trained model (i.e. yolov8n.pt) with the custom dataset obtained from Roboflow (i.e. data.yaml). The suggested value for "epochs=100" to obtain a more accurate model. This program performs:
    - Generates a model "yolo8n_custom.pt"
    - Evaluates the model performances
    
    ````python
    # train_model.py
    from ultralytics import YOLO

    model = YOLO("yolov8n.pt")

    model.train(
        data="data.yaml",
        epochs=30,
        imgsz=640,
        device="cpu"
    )

    # After training, use:
    # runs/detect/train/weights/best.pt
    # and copy the best.pt file to a /model folder with a proper name
    ````
- `2_detect_image.py`: Make prediction on image file using the saved custom model (i.e. yolov8n_custom.pt)

    ````python
    # This script demonstrates how to train a YOLOv8n model using the Ultralytics YOLO library.
    from ultralytics import YOLO

    # Load a pretrained YOLO8n model
    model = YOLO("yolov8n_custom.pt")  # Load the YOLOv8n model
    print(model.names)
    # Perform object detection on an image
    results = model("test/images/prohibido.jpg")  # Predict on an image from test set
    #results = model("Foto_.jpg")
    #results = model("Foto_2.jpg")
    results[0].show()  # Display results
    ````
    > `yolov8n_custom_en.pt` has label names in english
- `3_detect_video.py`: Make prediction on image file using the saved custom model (i.e. yolov8n_custom.pt)

### **Software** test in Gazebo: 

Bringup the robot in simulation:
````shell
ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml use_sim_time:=true x0:=0.5 y0:=-1.5 yaw0:=1.57 robot:=rubot/rubot_mecanum.urdf custom_world:=square4m_sign.world
````
Use the ``yolo_prediction_compressed_sw.py`` after the navigation node is launched.
````shell
ros2 run my_robot_ai_identification rt_prediction_yolo_exec
````
> You have to change the model path to `/home/user/ROS2_rUBot_mecanum_ws/src/AI_Projects/my_robot_ai_identification/models/yolov8n_custom.pt`

To see the image with prediction on RVIZ2, select a new Image message on topic /inference_result
    ![](./Images/07_Yolo/11_prediction_sw.png)
    ![](./Images/07_Yolo/11_prediction_sw2.png)

### **Hardware** Test in real LIMO robot:
You have to install on the Limo robot container:
````shell
apt update
apt install python3-pip
pip install ultralytics
# needed numpy version compatible
pip3 uninstall numpy
pip3 install "numpy<2.0"
#
apt install git
git clone https://github.com/manelpuig/ROS2_rUBot_mecanum_ws.git
source /opt/ros/humble/setup.bash
apt install python3-colcon-common-extensions
apt install build-essential
colcon build
source install/setup.bash
ros2 run my_robot_ai_identification rt_prediction_yolo_exec
````
    
Run The real-time prediction:
````shell
ros2 run my_robot_ai_identification rt_prediction_yolo_exec
````
> You have to change the model path to '/root/ROS2_rUBot_mecanum_ws/src/AI_Projects/my_robot_ai_identification/models/yolov8n_custom.pt
