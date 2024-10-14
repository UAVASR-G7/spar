# spar
A mid-level navigation software to implement flight actions with PX4 and ROS

These instructions on compiling, running, and available interfaces, mainly reference the `spar_node` 'Autopilot' qutas setup guides https://github.com/qutas/info/wiki/UAV-Setup-Guides-(2024)

Software Interface Setup: QGroundControl 
(1)	QGroundControl is used as part of the GCS for the mission. Before flight, several aspects, including parameters and the radio controller, must be configured to ensure successful and safe flight, listed below.
(2)	Airframe: Generic quadcopter is selected, as UAV is quadcopter
(3)	Sensors: Compass, gyroscope, accelerometer, and level horizon are calibrated and saved to a file, allowing this process to be done only once
(4)	Radio: Remote controller is connected to onboard radio and calibrated to allow user to take control of UAV
(5)	Flight Mode: flight mode is initially set to stabilised, and can be set to different modes depending on the type of flight (automatic take-off/land, return home, search, hold, lock altitude, etc)
(6)	Power: Battery 1 is set with the source power module of a 3S battery configuration, where each cell has an empty voltage of 3.4V and a full voltage of 4.2V. Specific parameters, voltage divider, amps per volt, and voltage drop on full load are measured and calculated before applying these values in QGroundControl
(7)	Autonomous flight settings - Localization: The autopilot autonomous system uses the Opti Track based localisation within the flight area, in order to configure QGC software to enable the Opti Track localisation change the following parameters within the advanced settings of QGC (Serna, 2024).

EKF2_EV_DELAY = 50  // Default 175
EKF2_AID_MASK = 24  // Enable Bits 3 and 4 (Vision Position and Vision Yaw Fusion) and Disable GPS Fusion
EKF2_HGT_MODE = 3   // Disable Barometric Height and Enable Vision Height
RTL_RETURN_ALT = 2  // These edit the failsafe mode to prevent crashing into the ceiling
RTL_DESCEND_ALT = 2

(8)	Autonomous flight settings – Enable autopilot to accept input from on-board computer using TELEM3 port with the following settings I QGC (Serna, 2024).

MAV_1_CONFIG = TELEM 3 // MAV_1_CONFIG is often used to map the TELEM 3 port to the correct settings
MAV_1_FLOW_CTRL = Force off // Sets flow control off, not wired in the current serial cable.
MAV_1_MODE = Onboard   // Sets the expected input type to be from an "onboard" system
SER_TEL3_BAUD = 921600 // A baud rate 921600 or higher recommended for on-board applications

By configuring QGroundControl in this way allows the UAV to operate as desired, successfully, and safely such that the system requirements and high-level objectives for this subsystem are met.  
5.3.2 Software Interface 2: ROS 

Initializing Mavlink (serial communications between flight controller and Pi4)
The on-board computer leverages the Robot Operating System (ROS) to integrate various subsystems into a cohesive platform. See the Image & Target Acquisition Preliminary Design and Operator Interface Preliminary Design reports for steps to initialise ROS installation and initial setup. 

Installing MAVROS for Mavlink Protocol:
After successfully installing ROS on the Linux operating system, the MAVROS plugin must be installed. MAVROS serves as a vital communication bridge between ROS and the MAVLink-compatible autopilot, specifically the Pix32v6. This installation enables the ROS framework to communicate effectively with the autopilot, facilitating control and data exchange. See the following first steps to installing and configuring the operating systems to run MAVROS (Serna, 2024).

(1)	Firstly, install the MAVROS plug in from the terminal window within the on-board computer using the following command. 

sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
(2)	Now install a set of geographic datasets library. 

roscat mavros install_geographiclib_datasets.sh | sudo bash

(3)	Ensure to enable serial adaptors (this enables serial communications between the USB-Serial converter, see hardware interface 7 and interface 1 above for more information), use the following commands to enable the setting and log out and in to reconfigure. 

sudo usermod -aG dialout $USER 
sudo apt remove modemmanager

(4)	Now within this application some packages that will be used for autonomous launch, observation and autopilot configuration. The spar and mavros packages come with a selection of launch files that can connect straight to the UAV, follow the preceding commands to configure the files within the directory. 

mkdir -p ~/catkin_ws/launch
cd ~/catkin_ws/launch
roscp qutas_lab_450 px4_flight_control.launch ~/catkin_ws/launch/control.launch
roscp mavros px4_pluginlists.yaml ./
roscp mavros px4_config.yaml ./

(5)	Now some of these plug-in files will need to be edited appropriately to work in conjunction with other subsystems configurations. Firstly edit the following lines, within the control.launch file (configured/created in previous steps) using the code to open the settings and save after editing, write the UAV name in place of uav_name characters/strings.

nano ~/catkin_ws/launch/control.launch
<arg name="uav_name" default="uav" />

(6)	Now for some more editing within control.launch, this command needs to know the location/directory which the installed plug ins are located, hence ensure the lines are changed as such. 

<arg name="pluginlists_yaml" value="/home/USERNAME/catkin_ws/launch/px4_pluginlists.yaml" />
<arg name="config_yaml" value="/home/USERNAME/catkin_ws/launch/px4_config.yaml" />

(7)	Now also within control.launch, MAVROS needs to know the baud rate for the autopilot (see QGC advanced parameter settings for reference in setting the baud rate for the TEL3 port). Set the baud rate by changing the code as follows.

<arg name="fcu_url" default="/dev/ttyACM0:57600" />
<arg name="fcu_url" default="/dev/ttyUSB0:921600" />

(8)	Lastly, the OptiTrack system, needs to be aware of the UAV team name settings, hence change the following where the UAV name replaces “cdrone”

<arg name="vrpn_object_name" default="cdrone" />

(9)	Finally, as a pre-flight check, run the following command and the terminal should exit with the following line/s. 

roslaunch qutas_lab_450 environment. Launch “(Opti-Track Localisation run)”
roslaunch ~/catkin_ws/launch/control.launch “(Autopilot Ready to arm then launch run)”

CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
[Spar] Received pose...
[Spar] Received mav_state...

Now the following commands can be used to take off, land and follow pre-set waypoints, which is all achieved through the spar node. 

rosrun spar_node takeoff
rosrun spar_node land
rosrun spar_node demo_wp_roi

By meticulously following the detailed installation and configuration procedures, the Raspberry Pi is appropriately configured to send serial data to the autopilot. This thorough setup process ensures that the system is ready for autonomous flight, with all necessary components properly synchronized and operational.

Now to update the nvagivation scritps to this repository, simply navigate to the spar workspace and clone this respoitory (which is forked from the main qutas repository) use git clone "(https://github.com/UAVASR-G7/spar.git)". 

Lastly, navigate to the catkin_ws location and run the execution catkin_make to rebuild the work-space. 


