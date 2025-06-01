import sys
import rospy
import moveit_commander

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('get_pose', anonymous=True)

    arm = moveit_commander.MoveGroupCommander("arm")
    print("Current pose:\n", arm.get_current_pose().pose)

if __name__ == '__main__':
    main()
