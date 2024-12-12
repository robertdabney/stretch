This is the repository for Johnson Engineering Center Autonomous Navigation by Robert Dabney at RPI

There are two packages: dabner_nav and dabner_interfaces

dabner_interfaces has a custom service defined that is used in dabner_nav

dabner_nav has three custom nodes: jec_nav, logic, speech

jec_nav interfaces with the nav2 navigation stack
logic controls the overall logic of the program
speech is a voice recognition node to act as an input to the system

This project was originally with the Stretch RE1 robot by Hello Robot, however, the Robot became unusable during the course of the project and a turtlebot3 simulator was used instead

To run the navigation demo, clone and build the repo using colcon_build
You will have to install ros2 humble, the turtlebot3 package, the Nav2 package, as well as pandas and pyaudio

Copy the world file to the turtlebot3 world file with the following command

	sudo cp dabner_ws/src/dabner_nav/worlds/jec6_world.world /opt/ros/humble/share/turtlebot3_gazebo/worlds/jec6_world.world

	You may have to "sudo chmod 777 jec6_world.world" after the copy in the turtlebot folder

Then we have three commands, make sure to source the dabner_ws first with the command "source dabner_ws/install/setup.bash"

	Launch the gazebo world: "ros2 launch dabner_nav turtlebot3_jec.launch.py"

	Launch the turtlebot navigation stack: "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=dabner_ws/src/dabner_nav/maps/jec6_simulated.yaml"

	Launch the custom nodes: "ros2 launch dabner_nav voice_demo_launch.py"

Then whenever the robot is not navigating, speak into a microphone a number 0-10 in accordance to "simulated_waypoints.csv" in the dabner_nav maps folder



The custom code is hardware agnostic, it should work with any Nav2 stack and microphone. Thus to get it working with the Stretch RE1, you can follow similar steps: launch the robot, launch the nav2 stack, launch the custom nodes.

The command to launch the stretch and nav2 stack is: "ros2 launch stretch_nav2 navigation.launch.py teleop_type:=keyboard map:=dabner_ws/src/dabner_nav/maps/jec6final.yaml"

Note that the waypoints would be different and a new waypoints.csv would need to be made and updated in jec_nav.py


To launch this with a real turtlebot, it should only require the command: "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=dabner_ws/src/dabner_nav/maps/jec6final.yaml"


To test this without the speech recognition, as this could be hardware dependent on your audio drivers, only run the navigation node: "ros2 run dabner_nav jec_nav"
	Then you can request a waypoint via terminal with the command: "ros2 service call /request_waypoint dabner_interfaces/srv/RequestNumber "{number: #}"", where # is the requested waypoint
