TurtleBot Arm
=============

This is just a test for simulating the Turtlebot Arm (more exact: the PhantomX Pincher Arm) in Gazebo, with MoveIt control. For a related question of mine see http://answers.gazebosim.org/question/18994/turtlebot-arm-gazebo-simulation-with-moveit/. The original repository is https://github.com/turtlebot/turtlebot_arm.

It currently doesn't use the arbotix drivers (as it should be, because they are needed when moving the real robot).

Currently working (more or less):
        roslaunch turtlebot_arm_gazebo turtlebot_arm_world.launch
        roslaunch turtlebot_arm_moveit_config demo_planning.launch
