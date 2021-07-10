# Robot Arm Control Using ROS
This Repository will explain my first task in Robotics and AI department at  [SMART METHODS](https://github.com/smart-methods) summer training.

## Task Requirements: 
  - Control a robot arm actuator using Robot Operating System (ROS) platform with Rviz, Gazebo and Moveit Simulator.  

## Steps:
  1. Install Ubuntu to work with ROS platform. I will use Ubuntu-18.04 (You can install Ubuntu with Windows in a virtual machine like Virtual box or Vmware).
  2. Instal [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) using the commands bellow in the terminal (Melodic is the version of ROS that correspond with Ubuntu-18.04).
     * `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
     * `sudo apt install curl` 
     * `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
     * `sudo apt-get update`
     * `sudo apt install ros-melodic-desktop-full`
     * `apt search ros-melodic`
     * `echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`
     * `source ~/.bashrc`
     * `sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`
     * `sudo apt install python-rosdep`
     * `sudo rosdep init`
     * `rosdep update`
3. Install a Catkin Workspace which is a folder to combine all packages for ROS.
   * `sudo apt-get install ros-melodic-catkin` 
   * `mkdir -p ~/catkin_ws/src`
   * `cd ~/catkin_ws/` 
   * `catkin_make`
   * `cd ~/catkin_ws/src`
   * `git clone https://github.com/smart-methods/arduino_robot_arm.git` we will use the arm from SM to work with.
   * `cd ~/catkin_ws`
   * `rosdep install --from-paths src --ignore-src -r -y`
   * `sudo apt-get install ros-melodic-moveit`
   * `sudo apt-get install ros-melodic-joint-state-publisher ros-kinetic-joint-state-publisher-gui`
   * `sudo apt-get install ros-melodic-gazebo-ros-control joint-state-publisher`
   * `sudo apt-get install ros-melodic-ros-controllers ros-melodic-ros-control`
   * open a file (bashrc) : `sudo nano ~/.bashrc`.
   * At the end of the file add the follwing line`source /home/mo7d_saleh/catkin_ws/devel/setup.bash` then press ctrl + o, (Note that mo7d_saleh is my ubuntu username).
   * `source ~/.bashrc`
   * To lunch the Rviz simulator with slider motors control (joint_state_publisher) use this command `roslaunch robot_arm_pkg check_motors.launch`
   * Rviz Simulator:
   
   ![Circuit Diagram](https://github.com/mo7ammed-saleh/Robot_Arm_Control_in_ROS/blob/main/Simulation%20imgs/Control%20Arn%20in%20Rviz%20.png)
   
4. To control the robot arm physically connect the circuit diagram with your arm and the install Arduino IDE and ros_lib.
   * Circuit Wiring:
   
   ![Circuit Diagram](https://github.com/mo7ammed-saleh/Robot_Arm_Control_in_ROS/blob/main/Simulation%20imgs/circuit.png) 
   
   * Install Arduino IDE software `wget https://downloads.arduino.cc/arduino-1.8.12-linux64.tar.xz` then `tar -xvf arduino-1.8.12-linux64.tar.xz` then `cd arduino-1.8.12/` then `sudo ./install.sh`
   * Download rosserial to communicate with arduino using the following commands `sudo apt-get install ros-melodic-rosserial-arduino` then `sudo apt-get install ros-melodic-rosserial`
   * Download ros_lib on arduino software using the following commands `cd Arduino/libraries` then `rm -rf ros_lib` then `rosrun rosserial_arduino make_libraries.py .`
   * To upload the Arduino code, select the Arduino port to be used on Ubuntu system, then we need to change the permissions (it might be ttyACM) by `ls -l /dev |grep ttyUSB` then `sudo chmod -R 777 /dev/ttyUSB0` then upload the code from Arduino IDE
   * Run Rviz `roslaunch robot_arm_pkg check_motors.launch`  `rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200`
  
5. Control the Arm in Gazebo simulator (real simulation).
    * Change the permission `cd catkin/src/arduino_robot_arm/robot_arm_pkg/scripts` then `sudo chmod +x joint_states_to_gazebo.py`
    * Open new terminal to launch Rviz `roslaunch robot_arm_pkg check_motors.launch`
    * Open new terminal to launch Gazebo `roslaunch robot_arm_pkg check_motors_gazebo.launch`
    * Then run the python script to communicate with Gazebo `rosrun robot_arm_pkg joint_states_to_gazebo.py` 
    * Rviz with Gazebo:
    
     ![Circuit Diagram](https://github.com/mo7ammed-saleh/Robot_Arm_Control_in_ROS/blob/main/Simulation%20imgs/Rviz%20with%20Gazebo%20Simulator.png)
     
    * Diiferan Angles:
    
    * ![Circuit Diagram](https://github.com/mo7ammed-saleh/Robot_Arm_Control_in_ROS/blob/main/Simulation%20imgs/different%20Angles.png)
    
   6. Now, lets used Moveit in Rvis which will help for kinematics, motion planning, trajectory processing and controlling the robot 
     * `roslaunch moveit_pkg demo.launch`
