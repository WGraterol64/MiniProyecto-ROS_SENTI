FROM ros:melodic

# Change to bash
SHELL ["/bin/bash", "-c"]

# Installing dependencies 
ENV DEBIAN_FRONTEND teletype
RUN apt-get update && yes | apt-get install -y ros-melodic-pepper-meshes \
	ros-melodic-gazebo-ros-control ros-melodic-rqt-joint-trajectory-controller \
	ros-melodic-moveit* ros-melodic-tf2-sensor-msgs ros-melodic-ros-control \
	ros-melodic-ros-controllers ros-melodic-gazebo-ros ros-melodic-gazebo-plugins \
	ros-melodic-controller-manager python3-wstool ros-melodic-gazebo*

# Create pepper bot directory
ENV PEPPER_DIR /usr/src/pepper_sim_ws
RUN mkdir -p ${PEPPER_DIR}/src

# Installing packages of Pepper from github
RUN git clone -b correct_chain_model_and_gazebo_enabled \
	https://github.com/awesomebytes/pepper_robot \
	${PEPPER_DIR}/src/pepper_robot

RUN git clone -b simulation_that_works \
	https://github.com/awesomebytes/pepper_virtual \
	${PEPPER_DIR}/src/pepper_virtual

RUN git clone https://github.com/awesomebytes/gazebo_model_velocity_plugin \
	${PEPPER_DIR}/src/gazebo_model_velocity_plugin

RUN git clone https://github.com/pal-robotics/ddynamic_reconfigure_python \
	${PEPPER_DIR}/src/ddynamic_reconfigure_python


# Compiling project
RUN sed -i 's/(math::Vector3/(ignition::math::Vector3d/g' \
	${PEPPER_DIR}/src/gazebo_model_velocity_plugin/src/gazebo_ros_model_velocity.cpp 

RUN source /opt/ros/melodic/setup.bash && \
	/opt/ros/melodic/bin/catkin_make_isolated -C ${PEPPER_DIR}

RUN echo -e "\nsource /opt/ros/melodic/setup.bash\n\
export ROS_PEPPER_SIM_WS=${PEPPER_DIR}\n\
source ${PEPPER_DIR}/devel_isolated/setup.bash" >> ~/.bashrc


# Running gazebo will cause an error, which we will resolve by modifying the ignition 
# services file
RUN timeout 10 gazebo 2> /dev/null || echo ""
RUN sed -i 's@https://api.ignitionfuel.org@https://api.ignitionrobotics.org@' \
	/root/.ignition/fuel/config.yaml


# =========================== ENVIRONMENTS AND LAUNCHES =========================== #
# Environment 1
COPY worlds/museum.world \
	${PEPPER_DIR}/src/pepper_virtual/pepper_gazebo_plugin/worlds
COPY launch/pepper_gazebo_plugin_museum.launch \
	${PEPPER_DIR}/src/pepper_virtual/pepper_gazebo_plugin/launch

# Environment 2

# Environment 3

# Environment 4


# =========================== SCRIPTS =========================== #
# Installing python3 dependences
RUN apt install -y python3-pip
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install opencv-python rospkg 
RUN python3 -m pip install pandas tensorflow 
RUN python3 -m pip install torch
RUN python3 -m pip install rdflib
RUN python3 -m pip install SpeechRecognition
RUN python3 -m pip install simple-term-menu 
RUN python3 -m pip install --ignore-installed sentence_transformers

# Move script
RUN source /opt/ros/melodic/setup.bash && \
	cd ${PEPPER_DIR}/src/ && \
	catkin_create_pkg pepper_move_controller geometry_msgs rospy && \
	cd ${PEPPER_DIR}/ && \
	catkin_make_isolated
COPY ./scripts/move.py ${PEPPER_DIR}/src/pepper_move_controller/src/

# Sensors scripts
COPY ./scripts/* ${PEPPER_DIR}/src/pepper_robot/pepper_sensors_py/nodes/

# Sound examples
COPY ./audio_test/ ${PEPPER_DIR}/src/pepper_robot/pepper_sensors_py/nodes/sounds/ 

# IA Models
COPY ./models/ ${PEPPER_DIR}/src/pepper_robot/pepper_sensors_py/nodes/models/

# Set execute permissions
RUN chmod u+x \
	${PEPPER_DIR}/src/pepper_robot/pepper_sensors_py/nodes/camera.py \
	${PEPPER_DIR}/src/pepper_move_controller/src/move.py \
	${PEPPER_DIR}/src/pepper_robot/pepper_sensors_py/nodes/selectAudio.py

# =========================== NPCS =========================== #
# Copy new NPCs
RUN mkdir ${PEPPER_DIR}/src/pepper_virtual/pepper_gazebo_plugin/models/npcs
COPY ./npcs/* ${PEPPER_DIR}/src/pepper_virtual/pepper_gazebo_plugin/models/npcs/

COPY ./launch/* ${PEPPER_DIR}/src/pepper_virtual/pepper_gazebo_plugin/launch/
COPY ./worlds/* ${PEPPER_DIR}/src/pepper_virtual/pepper_gazebo_plugin/worlds/

COPY ./launch/* ${PEPPER_DIR}/src/pepper_virtual/pepper_gazebo_plugin/launch/
COPY ./worlds/* ${PEPPER_DIR}/src/pepper_virtual/pepper_gazebo_plugin/worlds/

# =========================== MODELS =========================== #
COPY models/Facial/ ${PEPPER_DIR}/src/pepper_robot/pepper_sensors_py/nodes/

