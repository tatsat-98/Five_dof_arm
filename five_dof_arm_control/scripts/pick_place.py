#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place', anonymous=True)

    print("Available Planning Groups:", moveit_commander.RobotCommander().get_group_names())

    arm = moveit_commander.MoveGroupCommander("arm")
    print("Current pose:\n", arm.get_current_pose().pose)
    
    pose_target = geometry_msgs.msg.Pose()
    
    joint_goal = [0.0, 0.0, 0.0, 0.0]
    #arm.set_joint_value_target([joint1, joint2, joint3, joint4, joint5])
    arm.set_joint_value_target(joint_goal)

    # Plan and move
    success = arm.go(wait=True)
    if success:
        print("Motion to joint target successful.")
    else:
        print("Motion to joint target failed.")

    print("Planning Frame:", arm.get_planning_frame())
    print("End Effector Link:", arm.get_end_effector_link())
    print("Available Planning Groups:", moveit_commander.RobotCommander().get_group_names())
    print("Current pose:\n", arm.get_current_pose().pose)
    

    
    '''
    pose_target.orientation.w = 1.0
    pose_target.position.x = -0.6
    pose_target.position.y = 0.25
    pose_target.position.z = 0.9
    arm.set_pose_target(pose_target)
    '''
    print("Current pose:\n", arm.get_current_pose().pose)
    arm.go(wait=True)

    arm.stop()
    arm.clear_pose_targets()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    '''
if __name__ == '__main__':
    main() 
    '''