class CamNode():
    def __init__(self):
        self._publish_rate = rospy.get_param('~publish_rate', 100)                                                         # Get ROS parameters
        self.thresh = rospy.get_param('/roslaunch/threshold')
        self.fps = rospy.get_param('/roslaunch/fps')
        self.dist_spt = rospy.get_param('/roslaunch/distance_setpoint')

        self._result_pub = rospy.Publisher('~filtered_image', Image, queue_size=1)                                          # Create ROS topics
        self._camera_input = rospy.Subscriber('~input', Image, self.imageCallback, queue_size=1)
        scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)
        self.cmd_vel_pub = rospy.Publisher('/planner/cmd_vel', Twist, queue_size=10)
        self._joy_input = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)

        self.min_range = None
        self._last_msg = None                                                                                                # Create multitreading locks
        self._msg_lock = threading.Lock()
        self._vel_lock = threading.Lock()
        self.vel = Twist()

    def joy_callback(self, Joy):                                                                                   
        if self._vel_lock.acquire(False): 
            self.vel.linear.x  = 2*Joy.axes[1]
            self.vel.angular.z  = 2*Joy.axes[3]
            # self.cmd_vel_pub.publish(self.vel)
            self.get_logger().info("Velocity: Linear=%f" % (Joy.axes[1]))
            self._vel_lock.release()            

    def imageCallback(self, msg):
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._msg_lock.release()

    def callback_scan(self, data):
        fov = np.deg2rad(60)
        min_range = data.range_max
        min_idx = -1
        angle = data.angle_min
        for idx, r in enumerate(data.ranges):
            angle += data.angle_increment
            if -fov<angle<fov:
                if r<min_range:
                    min_range = r
                    min_idx = idx
        if min_idx < len(data.ranges)/2.0:
            self.direction = "RIGHT"
        else:
            self.direction = "LEFT"
        self.min_range = min_range

    def run(self):
        WIDTH = 64
        HEIGHT = 48
        if self.fps <= 0:
            FRAMERATE = 30
        else:
            FRAMERATE = self.fps        
        KEEP_BRAKE = 0.2*FRAMERATE                           # # Number of steps to continue backward output
        if self.thresh <= 0:
            THRESHOLD = 0.45                                 # Brake when the estimated TTC falls below this value
        else:
            THRESHOLD = self.thresh         
        R = 0.4                                              # Estimated TTC update ratio
        if self.dist_spt <= 0: 
            SPT = 0.4
        else:
            SPT = self.dist_spt        
        rate = rospy.Rate(self._publish_rate)                # Rate of the main loop
        cv_bridge = CvBridge()                               # ROS camera interface
        ctrl = CarController()                               # interface to PWM controlled motors on the car
        ctrl.steer(0)                                        # ahead
        count_loop = 0
        while not rospy.is_shutdown():                       # while ROS running grab the frames from the camera

            if self._msg_lock.acquire(False):                # If there is no lock on the message (not being written to in the moment)
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue
                  
            if msg is not None:                                                       # If the message is not empty
                np_image = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')                       # Convert the message to an OpenCV object
                img_resize = cv2.resize(np_image, (256, 256))
                img = img_resize[:HEIGHT].astype(np.float32)                          # I420->Y
                if count_loop == 0:
                    ettc = NaiveTTC(img, FRAMERATE, m0=5,r0=0.60)
                    ctrl.throttle(1.0)
                    brake = 0
                    mttc = 0.50		
                    count_loop += 1
                elif count_loop >= 1:
                    ttc = ettc.update(img)
                    if ttc>0:
                        mttc = min(1.0,(1-R)*mttc+R*ttc)
                    print("{0:+03.6f}".format(mttc))
                    if brake==0 and mttc<THRESHOLD:
                        brake = 1
                    elif brake>0:
                        if brake<KEEP_BRAKE:
                            ctrl.throttle(-1.0)                                          # slow down
                            if self._vel_lock.acquire(False): 
                                self.vel.linear.x = 0.05
                                self._vel_lock.release()
                            brake += 1
                        else:
                            break
                image_msg = cv_bridge.cv2_to_imgmsg(img, 'bgr8')
                self._result_pub.publish(image_msg)
            if self._vel_lock.acquire(False): 
                if self.min_range is not None:
                    if self.min_range >= SPT:
                        if not(brake<KEEP_BRAKE and brake>0):                             # no brake applied from the video feed
                            self.vel.linear.x = 0.2                                       # go forward x                              
                        ctrl.steer(0.0)                                                   # steering of piCar is ahead
                        self.vel.angular.z = 0.0
                    else:
                        vel.linear.x = 0.0
                        if self.direction == "RIGHT":
                            self.vel.angular.z = 0.5
                            ctrl.steer(-0.1)                                              # turn right
                        elif self.direction == "LEFT":
                            self.vel.angular.z = -0.5
                            ctrl.steer(0.1)                                               # turn left
                self.cmd_vel_pub.publish(self.vel)                                        # publish the twist to ROS   
                self._vel_lock.release()                    
        ctrl.brake()                                                                      # stop  
        if self._vel_lock.acquire(False):         
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.vel)                                                # publish the twist to ROS  
            self._vel_lock.release()            