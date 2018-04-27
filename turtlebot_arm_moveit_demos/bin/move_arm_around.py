#!/usr/bin/env python

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group_arm = moveit_commander.MoveGroupCommander("arm")
  group_gripper = moveit_commander.MoveGroupCommander("gripper")


  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group_arm.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ End effector: %s" % group_arm.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
  
  print(group_arm.has_end_effector_link())

  print(group_gripper.has_end_effector_link())
  
  
  group_variable_values = group_arm.get_current_joint_values()
  if(not group_variable_values.__contains__(0)):
    print "============ going to home position"
    # print "============ Generating plan 0"
    group_arm.clear_pose_targets()
    print "============ Joint values: ", group_variable_values
    for i in range(len(group_variable_values)):
      group_variable_values[i] = 0
    group_arm.set_joint_value_target(group_variable_values)
    plan0 = group_arm.plan()
    #print "============ Waiting while RVIZ displays plan0..."
    #rospy.sleep(5)
    # print "============ executing plan0..."
    group_arm.execute(plan0)
  
  # rospy.sleep(2)
  # # define a pose for the box (specified relative to frame_id)
  # box_pose = geometry_msgs.msg.PoseStamped()
  # box_pose.header.frame_id = robot.get_planning_frame()
  # box_pose.pose.orientation.w = 0.0
  # box_pose.pose.position.x = 1.0
  # box_pose.pose.position.y = 0.0
  # box_pose.pose.position.z = 1.5

  # Now, let's add the collision object into the world
  # print "============ Add an object into the world"
  # scene.remove_world_object("box1")
  # scene.add_box("box1", box_pose, size = (0.4, 0.4, 0.1))

  # # Sleep to allow MoveGroup to recieve and process the collision object message
  # rospy.sleep(2)

  # # Now when we plan a trajectory it will avoid the obstacle
  print "============ current pose:"
  #group_arm.allow_replanning(true)
  pose_current = group_arm.get_current_pose()
  print(pose_current)
  pose_target = pose_current.pose

  print "============ moving arm"
  # pose_target = geometry_msgs.msg.Pose()
  # pose_target.orientation.w = 1.5
  # pose_target.orientation.x = 5
  # pose_target.position.x = 0.7
  # pose_target.position.y = -0.05
  # pose_target.position.z = 1.1
  pose_target.position.z -= 0.05
  # when working with the orientation, the quaternion has to be normalized, so this doesn't work:
  # pose_target.orientation.x -= 0.5
  import tf
  #pose_target.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0.5,-0.1,0))
  # quaternion = tf.transformations.quaternion_from_euler(0, 2, 0)
  # print(quaternion)
  # #type(pose_target) = geometry_msgs.msg.Pose
  # pose_target.orientation.x = quaternion[0]
  # pose_target.orientation.y = quaternion[1]
  # pose_target.orientation.z = quaternion[2]
  # pose_target.orientation.w = quaternion[3]
  print(pose_target)
  group_arm.set_pose_target(pose_target)
  plan1 = group_arm.plan()

  #print "============ Waiting while RVIZ displays plan1..."
  #rospy.sleep(5)

  # print "============ executing plan1..."
  group_arm.execute(plan1)


  #rospy.sleep(5)
  #scene.remove_world_object("box1")
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL
  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
