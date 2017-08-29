#!/usr/bin/env python
import time
import klampt
import argparse
import struct
import sys
import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from ebolabot import *

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

rightx = 0.6569;
righty = -0.8859;
rightz = 0.03886;

leftx = 0.6575;
lefty = 0.85198;
leftz = 0.03883.

def getch():
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def ik_test(limb):
    rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=leftx,
                    y=lefty,
                    z=leftx,
                ),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=rightx,
                    y=righty,
                    z=rightz,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

        robot.left_limb.positionCommand(resp.joints[0].position)

        print "Response Message:\n", resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0

char = None

def keypress():
    global char
    char = getch()

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    #lrobot.setKlamptModel("/home/hauser/gitlibs/Klampt/data/robots/baxter_col.rob") #update
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    keepGoing = True
    while keepGoing:
        keypress()
        if char == "x":
    	       keepGoing = False
    	       print "Done"
    	       break
        elif char == "w":
            righty = righty + 0.05
            ik_test(args.limb)
        elif char == "s":
            righty = righty - 0.05
            ik_test(args.limb)
        elif char == "a":
            rightx = rightx - 0.05
            ik_test(args.limb)
        elif char == "d":
            rightx = rightx + 0.05
            ik_test(args.limb)
        elif char == "z":
            rightz = rightz - 0.05
            ik_test(args.limb)
        elif char == "q":
            rightz = rightz + 0.05
            ik_test(args.limb)
        else:
            print "Key pressed is " + char ". Not a command."

main()