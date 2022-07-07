
# from multiprocessing.connection import wait
import sys
import copy
from tokenize import group
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


print("============ Starting tutorial setup")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()


print("============ Setting move groups ============")
group_left = moveit_commander.MoveGroupCommander("left_arm")
# group_left.set_planner_id("ESTkConfigDefault")

group_right = moveit_commander.MoveGroupCommander("right_arm")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=10)


print("============ Left arm reference frame: %s ============" % group_left.get_planning_frame())
print("============ Left arm end effector link: %s ============" % group_left.get_end_effector_link())

print("============ Robot Groups: ============")
print(robot.get_group_names())

print("============ Printing robot state ============")
print(robot.get_current_state())
print("============")


print("============ Generating plan_left ============")
# group_variable_values = group_left.get_current_joint_values()
# print("GROUP VARIABLE VALUES:")
# print(group_variable_values)
# group_variable_values[0] = 1.0
# group_left.set_joint_value_target([-0.5,-0.5,-0.5,-0.5,1,1,1]
# )
# plan_left = group_left.plan()

print("============ Waiting while RVIZ displays plan_left... ============")
# rospy.sleep(10)

print("============ Visualizing plan_left ============")
# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
# display_trajectory.trajectory_start = robot.get_current_state()
# display_trajectory.trajectory.append(plan_left)
# display_trajectory_publisher.publish(display_trajectory)

print("============ Waiting while plan_left is visualized (again)... ============")
# rospy.sleep(5)

counter=0
while(counter<10):
	# group_left.set_joint_value_target([-0.9,-0.5,-0.5,-0.5,-0.5,0.5,1])
	# group_left.go(wait=True)
	# group_left.set_joint_value_target([-0.9,-0.5,-0.5,0.5,-0.5,-0.5,1])
	# group_left.go(wait=True)
    First_Pose=geometry_msgs.msg.Pose()
    First_Pose.orientation.w=1.0
    First_Pose.position.x = 0.2
    First_Pose.position.y = 0.6
    First_Pose.position.z = 0.7

    group_left.set_pose_target(First_Pose)
    plan=group_left.go(wait=True)
    group_left.stop()
    group_left.clear_pose_targets()


    # rospy.sleep(5)

    Second_Pose=geometry_msgs.msg.Pose()
    Second_Pose.orientation.w=1.0
    Second_Pose.position.x = 0.2
    Second_Pose.position.y = 0.3
    Second_Pose.position.z = 0.8

    counter+=1

    group_left.set_pose_target(Second_Pose)
    plan=group_left.go(wait=True)
    group_left.stop()
    group_left.clear_pose_targets()



# print("============ Generating plan_left ============")
# group_variable_values = group_right.get_current_joint_values()

# group_variable_values[0] = 1.0
# group_right.set_joint_value_target(group_variable_values)

# plan_right = group_right.plan()

# print("============ Waiting while RVIZ displays plan_left... ============")
# rospy.sleep(10)


# print("============ Visualizing plan_left ============")
# display_trajectory = moveit_msgs.msg.DisplayTrajectory()

# display_trajectory.trajectory_start = robot.get_current_state()
# display_trajectory.trajectory.append(plan_right)
# display_trajectory_publisher.publish(display_trajectory)

# print("============ Waiting while plan_left is visualized (again)... ============")
# rospy.sleep(5)

# group_right.go(wait=True)






moveit_commander.roscpp_shutdown()
