#!/usr/bin/env python3

# --- imports ---
import rospy
import geometry_msgs.msg
import tf
from math import sqrt, pi, sin, cos, tan
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringFeedback, SteeringAngle, SteeringCommand, Speed
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion


########### DECLARACION DE VARIABLES POSICION DEL ROBOT #############
x = 0.0
y = 0.0
z = 0.0
th = 0.0
phi = 0.0

x0 = 0.0
y0 = 0.0
z0 = 0.0
th0 = 0.0
phi0 = 0.0

########### DECLARACION DE VARIABLES POSICION DE LA TRAYECTORIA DESEADA #############
xd = 0.0
yd = 0.0
Dxd = 0.0
Dyd = 0.0
DDxd = 0.0
DDyd = 0.0
DDDxd = 0.0
DDDyd = 0.0
DDDDxd = 0.0
DDDDyd = 0.0

########### DECLARACION DE VARIABLES POSICION DE LA TRAYECTORIA #############
Dx = 0.0
Dy = 0.0
DDx = 0.0
DDy = 0.0

Dx0 = 0.0
Dy0 = 0.0

########### DECLARACION DE ANGULOS DE EULER #############
yaw=0.0
pitch=0.0
roll = 0.0
quaternion = 0.0

yaw0=0.0
pitch0=0.0
roll0 = 0.0
quaternion0 = 0.0

########## DECLARACION DE VARIABLES PARA LOS NODOS #########
###### PUBLICADORES ######## 
### Variables que se enviaran a los nodos del autominy(v y phi)
v_control = SpeedCommand()
phi_control = SteeringCommand()
###### SUSCRIPTORES ######## 
pose_msg = Pose()
#last_odom = Odometry()
phi_miny = SteeringAngle()

def callbackOptitrack(msg):
  global pose_msg
  pose_msg.position.x = -msg.pose.position.x
  pose_msg.position.y = -msg.pose.position.y
  pose_msg.position.z = msg.pose.position.z
  pose_msg.orientation.x = -msg.pose.orientation.x
  pose_msg.orientation.y = -msg.pose.orientation.y
  pose_msg.orientation.z = msg.pose.orientation.z
  pose_msg.orientation.w = msg.pose.orientation.w
  #print("datos recibidos optitrack",msg)
  
#def callbackOdom(msg):
#  global last_odom
#  last_odom = msg
  
def callbackPhi(msg):
  global phi_miny 
  phi_miny  = msg
  
def waitForFirstOptitrack():
  while not rospy.is_shutdown() and pose_msg is None:
    rospy.loginfo("%s: No initial optitrack message received. Waiting for message...",rospy.get_caller_id())
    rospy.sleep(1.0)

# --- main program ---
# In ROS, nodes are uniquely named. If two nodes with the same
# node are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our node so that multiple instances can
# run simultaneously.
#rospy.init_node("mocap_optitrack", anonymous=True)
rospy.init_node("optitrack_data", anonymous=True)
# create subscribers and publishers
#rospy.Subscriber("/sensors/odometry/odom", Odometry, callbackOdom, queue_size=100)
rospy.Subscriber("/mocap_node/Autominy/pose", PoseStamped, callbackOptitrack)
#rospy.Subscriber("/sensors/arduino/steering_angle", SteeringFeedback, callbackangulo, queue_size=100)
rospy.Subscriber("/sensors/steering", SteeringAngle, callbackPhi, queue_size=100)



pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=100)
pub_steering = rospy.Publisher("/actuators/steering", SteeringCommand, queue_size=100)
#pub_omnibot = rospy.Publisher("/optitrack_data/info", Pose, queue_size=10)
#pub_info = rospy.Publisher("optitrack_data/info", String, queue_size=100)

rospy.loginfo(rospy.get_caller_id() + ": started!")

waitForFirstOptitrack()
# create a file to store data
archivo = open("/home/intelnuc/autominy/catkin_ws/src/simple_drive_control/pruebasAckermanExtendido/ackermanExtendido_bernulli_13.txt","w")
#archivo = open("/home/intelnuc/autominy/catkin_ws/src/simple_drive_control/pruebasAckermanExtendido/ackermanExtendido_circulo_2.txt","w")


######### LIMITACIONES DEL ROBOT #######
anglei = 0.0
velmax = 1.0#2.5
velmin = 0.07#-2.5
anglephimax = 0.38#0.3552#0.512
anglephimin = -0.4#-0.3428#-0.498

######## VARIABLES DE CONTROL #######
rx = 0.0
ry = 0.0
v = 0.07
vc = 0.0 
w = 0.0
wa = 0.0
u2 = 0.0
u2a = 0.0
u1 = 0.0
u1a = 0.0
C1 = 0.0
C2 = 0.0

ts = 6.0

z = 0.7
wn = 3/(z*ts)
p2 = 2.0*z*wn
p3 = wn*wn
p1 = 5.0
k0 = p1*p3
k1 = p3+p1*p2
k2 = p1+p2

#k0 = 0.1
#k1 = 2.0
#k2 = 1.0

#zy = 1.0
#wny = 4/(zy*ts)
#py2 = 2.0*zy*wny
#py3 = wny*wny
#py1 = 7.0

#ky0 = py1+py3
#ky1 = py3+py1*py2
#ky2 = py1+py2

l = 0.26

########## VARIABLES DE LA TRAYECTORIA #############
#rate = rospy.Rate(10) # 100hz = 0.01s
T = 100.0
#T = 70
tmuestreo = 0.02
Dt=tmuestreo
t = 0.0
rate = rospy.Rate(50)
  
# send commands - initial values
v_control.value = v
pub_speed.publish(v_control)
phi_control.value = anglei
pub_steering.publish(phi_control)


while not rospy.is_shutdown() and t <= 2.0*T :  
  #t = t2-t0
  #Dt = t2-t1
  print("tiempo : ",  t)
  #if (Dt>=tmuestreo):
  ##### Leer la posicion del autominy ####
  x = pose_msg.position.x
  y = pose_msg.position.y
  z = pose_msg.position.z
  quaternion = [pose_msg.orientation.x,pose_msg.orientation.y,pose_msg.orientation.z,pose_msg.orientation.w]
  (yaw,pitch,roll) = euler_from_quaternion(quaternion)
  phi= phi_miny.value
  th = roll
  
  if th>=0 and th<=pi:
    th=th
  else:
    th = th + 2*pi
    
  print("x : ", x)
  print("y : ", y)
  print("k0 : ", k0)
  print("k1 : ", k1)
  print("k2 : ", k2)
  #print("theta : ", th*180/pi)
  #print("phi : ", phi*180/pi)
  
  
  Dx = v*cos(th)
  Dy = v*sin(th)
  
  DDx = u1*cos(th)-v*v*tan(phi)*sin(th)/l
  DDy = u1*sin(th)+v*v*tan(phi)*cos(th)/l  

  ############ TRAYECTORIA DESEADA ##############
  #r = 1.3
  #w = 2.0*pi/T
  #xd = r*cos(w*t)
  #Dxd = -r*w*sin(w*t)
  #DDxd = -r*w*w*cos(w*t)
  #DDDxd = r*w*w*w*sin(w*t)
  
  #yd = r*sin(w*t)
  #Dyd = r*w*cos(w*t)
  #DDyd = -r*w*w*sin(w*t)
  #DDDyd = -r*w*w*w*cos(w*t)
  
  
  ####### Lenminiscata ####3333
  #w = (2.0*pi/T)
  #a = 2.0
  #b = 1.0
  #xd = a*cos((2*pi/T)*t)/(sqrt(1.0+sin((2*pi/T)*t)*sin((2*pi/T)*t)))
  #Dxd = -a*(w*sin(w*t)*cos(w*t)*cos(w*t)/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t)))))-(a*w*sin(w*t)/(sqrt((1.0+sin(w*t)*sin(w*t)))))
  #DDxd =-a*(w*w*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))))+(3*w*w*a*sin(w*t)*sin(w*t)*cos(w*t))/(sqrt((1+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))+(3.0*w*w*a*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))-(a*w*w*cos(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))
  #DDDxd = (a*w*w*w*sin(w*t))/(sqrt(1.0+sin(w*t)*sin(w*t)))+(a*w*w*w*sin(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))+(9.0*w*w*w*a*sin(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))-(3.0*w*w*w*a*sin(w*t)*sin(w*t)*sin(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))-(15.0*w*w*w*a*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))+(9.0*w*w*w*a*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))-(18.0*w*w*w*a*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))

  #yd = b*a*sin((2*pi/T)*t)*cos((2*pi/T)*t)/(sqrt(1.0+sin((2*pi/T)*t)*sin((2*pi/T)*t)))
  #Dyd = b*((a*w*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))))-(a*w*sin(w*t)*sin(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))))-(a*w*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t))/  (sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t)))))
  #DDyd = b*(-(4.0*w*w*a*cos(w*t)*sin(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))))-(3.0*w*w*a*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))+(3.0*w*w*a*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))+(3.0*w*w*a*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t)))))
  #DDDyd = b*((4.0*w*w*w*a*sin(w*t)*sin(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))))-(4.0*w*w*w*a*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))))+(22.0*w*w*w*a*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))-(3.0*w*w*w*a*cos(w*t)*cos(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))+(18.0*w*w*w*a*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))-(18.0*w*w*w*a*sin(w*t)*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))-(15.0*w*w*w*a*sin(w*t)*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t)*cos(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))))-(3.0*w*w*w*a*sin(w*t)*sin(w*t)*sin(w*t)*sin(w*t))/(sqrt((1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t))*(1.0+sin(w*t)*sin(w*t)))))
 
  w=2.0*pi/T;
  a = 2.3;
  d = 2.5;

  xd = a*cos(w*t)/(1+(sin(w*t)*(sin(w*t))));
  
  Dxd = -a*((w*sin(w*t)/(1+sin(w*t)*sin(w*t)))+2*w*sin(w*t)*cos(w*t)*cos(w*t)/((1+(sin(w*t)*sin(w*t)))*(1+(sin(w*t)*sin(w*t)))));
  
  DDxd = a*(-(w*w*cos(w*t)/(1+sin(w*t)*sin(w*t)))+(6*w*w*sin(w*t)*sin(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))-(2*w*w*cos(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))+(8*w*w*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t)))));
  
  DDDxd = a*((w*w*w*sin(w*t)/(1+sin(w*t)*sin(w*t)))+(20*w*w*w*sin(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))-(6*w*w*w*sin(w*t)*sin(w*t)*sin(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))-(48*w*w*w*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))+(24*w*w*w*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))-(48*w*w*w*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t)))));
  
  

  yd = d*(sin(w*t)*cos(w*t))/(1+(sin(w*t)*sin(w*t)));
  
  Dyd = d*((w*cos(w*t)*cos(w*t)/(1+sin(w*t)*sin(w*t)))-(w*sin(w*t)*sin(w*t)/(1+sin(w*t)*sin(w*t)))-(2*w*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t)))));
  
  DDyd = d*((-4*w*w*sin(w*t)*cos(w*t)/(1+sin(w*t)*sin(w*t)))-(6*w*w*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))+(6*w*w*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))+(8*w*w*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t)))));
  
  DDDyd = d*((-4*w*w*w*cos(w*t)*cos(w*t)/(1+sin(w*t)*sin(w*t)))+(4*w*w*w*sin(w*t)*sin(w*t)/(1+sin(w*t)*sin(w*t)))+(44*w*w*w*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))-(6*w*w*w*cos(w*t)*cos(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))-(6*w*w*w*sin(w*t)*sin(w*t)*sin(w*t)*sin(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))+(48*w*w*w*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))-(48*w*w*w*sin(w*t)*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))))-(48*w*w*w*sin(w*t)*sin(w*t)*sin(w*t)*sin(w*t)*cos(w*t)*cos(w*t)*cos(w*t)*cos(w*t)/((1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t))*(1+sin(w*t)*sin(w*t)))));

  # error
  ex = x-xd
  ey = y-yd
  
  Dex = Dx-Dxd
  Dey = Dy-Dyd  
  
  DDex = DDx-DDxd
  DDey = DDy-DDyd  
  
  
  # auxiliar control
  rx = -k0*ex -k1*Dex -k2*DDex +DDDxd
  ry = -k0*ey -k1*Dey -k2*DDey +DDDyd
  
  B1 = -(3.0*v*u1*sin(th)*tan(phi)/l)-v*v*v*cos(th)*tan(phi)*tan(phi)/(l*l)
  B2 = (3.0*v*u1*cos(th)*tan(phi)/l)-v*v*v*sin(th)*tan(phi)*tan(phi)/(l*l)
  
  C1 = rx - B1
  C2 = ry - B2
    
  # control
  u2 = cos(th)*C1+sin(th)*C2
  u1 = u1+Dt*(u2+u2a)/2.0
  v = v+Dt*(u1+u1a)/2.0
  
  w = -((l*cos(phi)*cos(phi)*sin(th))/(v*v))*C1+((l*cos(phi)*cos(phi)*cos(th))/(v*v))*C2
  anglei = anglei+Dt*(w+wa)/2.0

    
  # saturation
  if v >= velmax:
    v = velmax
    
  if v <= velmin:
    v = velmin
    
  if anglei >= anglephimax:
    anglei = anglephimax

  if anglei <= anglephimin:
    anglei = anglephimin
  
  print("ex : ", ex)
  print("ey : ", ey)

  # send commands
  v_control.value = v
  pub_speed.publish(v_control)
  phi_control.value = anglei
  pub_steering.publish(phi_control)

  # save data
  archivo.write(str(t) + " " + str(xd) + " " + str(yd) + " " + str(x) + " " + str(y) + " " + str(v) + " " + str(w) + " " + str(anglei) + " " + str(th) + " " + str(phi) + " " + str(ex) + " " + str(ey) + " " + str(z) + " " + str(ts) + "\n")

  t = t + tmuestreo
  wa = w
  u2a = u2
  u1a = u1
  x0 = x
  y0 = y
  Dx0 = Dx
  Dy0 = Dy
  rate.sleep()


# close the file
archivo.close()
# send commands - final values
v_control.value = 0.0
pub_speed.publish(v_control)
phi_control.value = 0.0
pub_steering.publish(phi_control)



# spin() simply keeps python from exiting until this node is stopped
#rospy.spin()
