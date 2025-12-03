# ROS2_Construct_Rover

This project was developed as I was learning to use ROS2, it showcases all the fundemental capabilities of ROS2 Humble middleware in python. Code was developed on a web-based virtual machine using "https://app.theconstruct.ai/", which has built-in features such as Gazebo, as well as an evaluation tool to validate my code, which is why you may notice some funny details (and this explain why I had to follow cetain naming conventions i.e. "quiz"). I have detailed the function of each directory, more specifically the nodes below:

- `services_quiz_srv`
  - `turn_s_server.py`
    - receives `/turn` message
    - converts message to `twist`
  - `turn_s_client.py`
    - calls `/turn` service

- `actions_quiz`
  - `actions_quiz_sever.py`
    - offers distance action
    - sets up Nav2 localizations
    - call Nav2 `NavigateToPose` action
    - tracks `/odom` -> calulates `distance_left` -> publishes data to client
  - `actions_quiz_client.py`
    - sends goal to action server
    - logs extracted `distance_left` feedback from server
   
- `my_action_server`
  - `action_server.py`
    - initialize localization
    - call Nav2 server to navigate to goal
   
- `my_action_client`
  - `action_client.py` (used for testing)
    - sends goal (in `seconds`) for rotation time
    - logs result
  - `custom_action_client.py`
    - gets meteor coordinates
    - navigates to initial goal using `send_goal`
    - `odom_callback` overrides goal if in range of meteor
   
- `basics_ros2_multithreading`
  - `scripts`
    - `function.py`: status logger with no ros2 callbacks
    - `callback_fucntion.py`: status logger that uses `rclpy.spin(node)`
    - `callback_spinonce_function.py`: status logger that used `rclpy.spin_once(node)`
    - `greentest.py`: opens OpenCV window with HSV trackbars to highlight "green"
    - `green_detector_node.py`: runs `PlantDetectorNode` (to rotate robot for up to 10 secs until plant is detected), when `GreenDetectorNode` service is called, green is highlighted using `MultiThreadedExecutor` to avoid conflicts
    - `plant_detector.py`: has `PlantDetectorNode'
    - `plant_detector_multithreading.py`: has two threads
    - `plant_detector_multithreading_callbackgroups.py`: actually declares mutally exclusive callback groups
   
- `mars_rover_tasks`
  - `subscriber_obstacle_detector.py`
    - subscribes to `laser_scan` and breaks sensor data into sectors
    - logs action based on detection in sectors
  - `publish_mars_rover_move.py`
    - test node to move rover
  - `autonomous_exploration.py`
    - sectored bostacle detection
    - `/odom` position tracking
    - wanders till certain distance from origin, then returns to (0,0)
  - `plant_detector.py`
    - uses `resnet18` model from `torchvision.models` for plant detection
  - `plant_detector_node.py`
    - subscribes to camera & odom
    - `PlantDetector` sees plant, pose is published (https://bitbucket.org/theconstructcore/basic_ros2_extra_files/src/main/)
  - `mars_rover_status_s_server.py` & `mars_rover_status_s_client.py`
    - `Trigger` service communication for mock robot status loggin
  - `text_recog.py`: OCR
  - `text_recog_node.py`: OCR subscriber node
  - `text_recog_s_server.py`: client to log text detection
  - `text_recog_s_client.py`: service added for text detection
  - `text_recog_s_server_custom.py`: can OCR custom a label
  - `text_recog_s_client_custom.py`: with bounding boxes

- `mars_rover_systems`
  - `heartbeat.py`
    - loggin timer callbacks
    - multiple entrypoints for multiple nodes using the same class
  - `temperature_monitor.py`
    - timer callback with warning logs

- `custom_interfaces` `services_quiz_srv` `actions_quiz_msg`
  - custom msg/srv/action files
   
- `logs_test`
  - `logger_example.py`
    - test/demo ROS2 logging levels
