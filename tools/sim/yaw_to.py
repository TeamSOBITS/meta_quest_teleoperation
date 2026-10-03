import rclpy,sys,math,time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
rclpy.init(); n=rclpy.create_node('yaw_to'); target=float(sys.argv[1]); yaw=[None]
def cb(m):
    q=m.pose.pose.orientation; yaw[0]=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
n.create_subscription(Odometry,'/sobit_home/odom',cb,10); p=n.create_publisher(Twist,'/sobit_home/cmd_vel',10)
t0=time.time()
while time.time()-t0<30:
    rclpy.spin_once(n,timeout_sec=0.05)
    if yaw[0] is None: continue
    e=math.atan2(math.sin(target-yaw[0]),math.cos(target-yaw[0]))
    t=Twist()
    if abs(e)<0.02: break
    t.angular.z=max(-0.3,min(0.3,1.0*e)); p.publish(t)
for i in range(5): p.publish(Twist()); time.sleep(0.1)
time.sleep(1.5); rclpy.spin_once(n,timeout_sec=0.2); print('yaw',yaw[0])
