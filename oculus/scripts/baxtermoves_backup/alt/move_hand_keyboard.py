import baxter_interface
import rospy
import ik_solver
from geometry_msgs.msg import (    
    Point,
    Quaternion,
)


rospy.init_node('move_left_hand')
rs = baxter_interface.RobotEnable()
rs.enable()

left = baxter_interface.Limb('right')
pose = left.endpoint_pose()
b = True
pos = pose.popitem()
orient = pose.popitem()
prev = pos[1]

left.set_joint_position_speed(1.0)

while b == True:
  print "-----------------------------------"
  pose = left.endpoint_pose()
  print "current pose: ", pose
  s = raw_input("Press q + enter to disable robot, x + enter to give new target ... ")
  
  if s == 'q':
    b = False
  elif s == 'x':    
    print "Enter new target location."
    s = raw_input("x: ")
    
    if s == '':
      x = prev.x
    else:
      x = float(s)      
      
    s = raw_input("y: ")
    
    if s == '':
      y = prev.y
    else:
      y = float(s)
    
    s = raw_input("z: ")
    
    if s == '':
      z = prev.z
    else:
      z = float(s)
      
    loc = Point(x,y,z)
    print "loc: ", loc    
    print "orient: ", orient[1]
    limb_joints = ik_solver.ik_solve('right', loc, orient[1])
    prev = loc
    
    if limb_joints == -1:
      continue
    
    print "limb_joints: ", limb_joints
    print "moving arm to limb_joints joints"
    left.move_to_joint_positions(limb_joints)

rs.disable()

#~ import roslib
#~ import baxter_interface
#~ import rospy
#~ from geometry_msgs.msg import (
    #~ PoseStamped,
    #~ Pose,
    #~ Point,
    #~ Quaternion,
#~ )
#~ import iodevices
#~ import ik_solver

#~ 
#~ roslib.load_manifest('joint_position')
#~ rospy.init_node("move_hand")
#~ rs = baxter_interface.RobotEnable()
#~ rs.enable()
#~ 
#~ left = baxter_interface.Limb('left')
#~ pose = left.endpoint_pose()
#~ print "left.endpoint_pose(): ", pose
#~ 
#~ jnames = left.joint_names()
#~ print "joint names: ", jnames
#~ jcurr = left.joint_angles()
#~ print "current joint angles: ", jcurr
#~ b = True
#~ 
#~ pos = pose.popitem()
#~ orient = pose.popitem()
#~ prev = pos[1]
#~ 
#~ while b == True:
  #~ print "-----------------------------------"
  #~ pose = left.endpoint_pose()
  #~ print "current pose: ", pose
  #~ s = raw_input("Press q + enter to disable robot, x + enter to give new target ... ")
  #~ 
  #~ if s == 'q':
    #~ b = False
  #~ elif s == 'x':    
    #~ print "Enter new target location."
    #~ s = raw_input("x: ")
    #~ 
    #~ if s == '':
      #~ x = prev.x
    #~ else:
      #~ x = float(s)      
      #~ 
    #~ s = raw_input("y: ")
    #~ 
    #~ if s == '':
      #~ y = prev.y
    #~ else:
      #~ y = float(s)
    #~ 
    #~ s = raw_input("z: ")
    #~ 
    #~ if s == '':
      #~ z = prev.z
    #~ else:
      #~ z = float(s)
      #~ 
    #~ loc = Point(x,y,z)
    #~ print "loc: ", loc    
    #~ print "orient: ", orient[1]
    #~ limb_joints = ik_solver.ik_solve('left', loc, orient[1])
    #~ prev = loc
    #~ 
    #~ if limb_joints == -1:
      #~ continue
    #~ 
    #~ print "limb_joints: ", limb_joints
    #~ print "moving arm to limb_joints joints"
    #~ left.move_to_joint_positions(limb_joints)
        #~ 
#~ print "Disabling robot... "
#~ rs.disable()
