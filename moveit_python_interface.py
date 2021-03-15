#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
#----------------------------------------------------------------------------------
from std_msgs.msg import Int32 # Messages used in the node must be imported.
from geometry_msgs.msg import Pose # new import for opencv communication

#----------------------------------------------------------------------------------
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInteface_RoboticArm_Rescue(object): 
  """MoveGroupPythonInteface_RoboticArm_Rescue"""

  def callback1(self):
    self.x = geometry_msgs.msg.Pose.position.x
    self.y = geometry_msgs.msg.Pose.position.y
    rospy.loginfo("Recieve data from x: %f, y: %f", geometry_msgs.msg.Pose.position.x,geometry_msgs.msg.Pose.position.y)
    return self.x, self.y
  
  
  def __init__(self):
    super(MoveGroupPythonInteface_RoboticArm_Rescue, self).__init__() 
 
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
#-------------------------------------------------------------------
    rospy.init_node('move_group_python_interface',
                    anonymous=True)
    rospy.Subscriber("follow_blob",Pose,self.callback1)
#-------------------------------------------------------------------

    ## Get the name of the robot - this will be used to properly define the end-effector link when adding a box
    robot_name = rospy.get_param("~robot_name")
    self.robot_name = robot_name

    ## Get the dof of the robot - this will make sure that the right number of joints are controlled
    dof = rospy.get_param("~dof")
    self.dof = dof

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Interbotix
    ## arm so we set ``group_name = interbotix_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Interbotix Arm:
    group_name = "interbotix_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher("move_group/display_planned_path",
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

   
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self, joint0, joint1, joint2, joint3, joint4):
  
    group = self.group
    dof = self.dof
    
    ## Planning to a Joint Goal
   
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = joint0      
    joint_goal[1] = joint1
    joint_goal[2] = joint2 
    joint_goal[3] = joint3
    if dof >= 5:
        joint_goal[4] = joint4
    if dof >= 6:
        joint_goal[5] = joint6

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END

    # For testing:
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def scoop_left(self,j0, j1, j2, j3, j4):
    Movement = MoveGroupPythonInteface_RoboticArm_Rescue()# recognizing because class must be initialized again 
   # Joint initialization
#------------------------- Saving object from water on the Left side of container---------------------- 
    j4 = -pi/2
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j3 = -pi/2
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j0=pi/4
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j4=0
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j3=0
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j0= -pi/2
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j3 = -pi/2
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j4 = -pi/2
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    return

  def scoop_right(self,j0, j1, j2, j3, j4):

    Movement = MoveGroupPythonInteface_RoboticArm_Rescue()# recognizing because class must be initialized again ********
   # Joint initialization
#------------------------- Saving object from water on the rightside of container---------------------- 
    j4 = pi/2
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j3 = -pi/2
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j0=-pi/3
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j4=0
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j3=0
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j0= -pi/2
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j3 = -pi/2
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    j4 = -pi/2
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)
    return


  def go_to_pose_goal(self):
    group = self.group
    robot_name = self.robot_name
    
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    if robot_name == "px100":
        pose_goal.position.x = 0.1
        pose_goal.position.z = 0.15
    else:
        pose_goal.position.x = 0.15
        pose_goal.position.z = 0.24
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    # For testing:
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self,d_x,d_y,d_z):
    group = self.group
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    scale = 0.01
    waypoints = []
    
    wpose = group.get_current_pose().pose
    wpose.position.y += scale*d_y 
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z -= scale * d_z  
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale*d_x  
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END
  def display_trajectory(self, plan):

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END

  def execute_plan(self, plan):
    group = self.group

    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:

    group.execute(plan, wait=True)
    
    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END

def main():
  try:
    print "============ Press `Enter` to begin Water Rescue (press ctrl-d to exit) ..."
    raw_input()
    Movement = MoveGroupPythonInteface_RoboticArm_Rescue()

    
    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    j0 = 0
    j1 = -pi/6
    j2 = pi/6
    j3 = -pi/3 
    j4 = 0
    
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)

    #raw_input()
    #Movement.go_to_pose_goal()

    print "============ Press `Enter` to plan and display a Cartesian path ..."
    raw_input()
    # Input coordinates from Opencv
    dx = float(input("Enter coordinate x : "))
    dy = float(input("Enter coordinate y: ")) 
    dz = 4
    
    cartesian_plan, fraction = Movement.plan_cartesian_path((-1)*dx,dy,dz)

    print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    #raw_input()
    Movement.display_trajectory(cartesian_plan)

    print "============ Cartesian Path to object ..."
    # Cartesian path to object 
    Movement.execute_plan(cartesian_plan) 

     #Run the scooping function
    print "============ Press 'Enter' to scoop the object"
    
    # Scooping right or left based on the y coordinate of the object
    raw_input()
    if dy<0:
       group = Movement.group
       joint_goal = group.get_current_joint_values()
       j0 = joint_goal[0]
       j1 = joint_goal[1]
       j2 = joint_goal[2]
       j3 = joint_goal[3]
       j4 = joint_goal[4]
       Movement.scoop_right( j0, j1, j2, j3, j4)
    else:
       group = Movement.group
       joint_goal = group.get_current_joint_values()
       j0 = joint_goal[0]
       j1 = joint_goal[1]
       j2 = joint_goal[2]
       j3 = joint_goal[3]
       j4 = joint_goal[4]
       Movement.scoop_left( j0, j1, j2, j3, j4)

    # go back to initial position
    j0 = 0
    j1 = -pi/6
    j2 = pi/6
    j3 = -pi/3 
    j4 = 0
    
    Movement.go_to_joint_state(j0, j1, j2, j3, j4)

    print "============ Trajectory complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
