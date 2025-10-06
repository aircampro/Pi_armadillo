class CamNode(Node): 
    def __init__(self):
        super().__init__('robo_picar_controls') 
        self.cmd_vel_pub = self.create_publisher(Twist,'cmd_vel', 10)
        self._result_pub = self.create_publisher(Image, '~filtered_image', 10)                                 # Create ROS topics
        self._camera_input = self.create_subscription(Image, '~input', self.imageCallback, 10)
        scan_sub = self.create_subscription(LaserScan, '/scan', self.callback_scan, 10)
        self.j_subscription = self.create_subscription(Joy,'joy', self.joy_callback, 10)                                  # we have added control from the joystick
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5                                                                                                # seconds for interrupt timer running action code
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vel = Twist()
        self.min_range = None
        self._last_msg = None                                                                                             # Create multitreading locks
        self._msg_lock = threading.Lock()
        self._vel_lock = threading.Lock()
        self._run_lock = threading.Lock()
        self.running = True
        self.count_loop = 0
        self.brake = 0

    def joy_callback(self, Joy): 
        if self._run_lock.acquire(False): 
            if self.running == True:                                                                                       # we didnt hit STOP altogether yet
                if self._vel_lock.acquire(False): 
                    self.vel.linear.x  = 2*Joy.axes[1]
                    self.vel.angular.z  = 2*Joy.axes[3]
                    # self.cmd_vel_pub.publish(self.vel)
                    self.get_logger().info("Velocity: Linear=%f" % (Joy.axes[1]))
                    self._vel_lock.release()            
            self._run_lock.release()  

    def imageCallback(self, msg):
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._msg_lock.release()

    def timer_callback(self):
        # now check how close it is and slow down if needed
        WIDTH = 64
        HEIGHT = 48
        FRAMERATE = 30      
        KEEP_BRAKE = 0.2*FRAMERATE                           # # Number of steps to continue backward output
        THRESHOLD = 0.45                                     # Brake when the estimated TTC falls below this value
        R = 0.4                                              # Estimated TTC update ratio
        SPT = 0.4      
        cv_bridge = CvBridge()                               # ROS camera interface
        ctrl = CarController()                               # interface to PWM controlled motors on the car
        ctrl.steer(0)                                        # ahead
        if self.running == True:
            if self._msg_lock.acquire(False):                # If there is no lock on the message (not being written to in the moment)
                msg2 = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                continue           
            if msg2 is not None:                                                       # If the message is not empty
                np_image = cv_bridge.imgmsg_to_cv2(msg2, 'bgr8')                       # Convert the message to an OpenCV object
                img_resize = cv2.resize(np_image, (256, 256))
                img = img_resize[:HEIGHT].astype(np.float32)                          # I420->Y
                if self.count_loop == 0:
                    ettc = NaiveTTC(img, FRAMERATE, m0=5,r0=0.60)
                    ctrl.throttle(1.0)
                    self.brake = 0
                    mttc = 0.50		
                    self.count_loop += 1
                elif self.count_loop >= 1:
                    ttc = ettc.update(img)
                    if ttc>0:
                        mttc = min(1.0,(1-R)*mttc+R*ttc)
                    print("{0:+03.6f}".format(mttc))
                    if self.brake==0 and mttc<THRESHOLD:
                        self.brake = 1
                    elif self.brake>0:
                        if self.brake<KEEP_BRAKE:
                            ctrl.throttle(-1.0)                                          # slow down
                            if self._vel_lock.acquire(False):
                                self.vel.linear.x = 0.05
                                self._vel_lock.release()
                            self.brake += 1
                        else:
                            ctrl.brake()                                                 # stop completely  
                            if self._vel_lock.acquire(False):                            
                                self.vel.linear.x = 0.0
                                self.vel.angular.z = 0.0 
                                if self._run_lock.acquire(False): 
                                    self.running = False
                                    self._run_lock.release()
                                self._vel_lock.release()
                image_msg = cv_bridge.cv2_to_imgmsg(img, 'bgr8')
                self._result_pub.publish(image_msg)

            if self.min_range is not None:
                if self.min_range >= SPT:
                    if self._vel_lock.acquire(False):    
                        if not(self.brake<KEEP_BRAKE and self.brake>0):                   # no brake applied from the video feed
                            self.vel.linear.x = 0.2                                       # go forward x                              
                        ctrl.steer(0.0)                                                   # steering of piCar is ahead
                        self.vel.angular.z = 0.0
                        self._vel_lock.release()
                else:
                    if self._vel_lock.acquire(False):
                        self.vel.linear.x = 0.0
                        if self.direction == "RIGHT":
                            self.vel.angular.z = 0.5
                            ctrl.steer(-0.1)                                              # turn right
                        elif self.direction == "LEFT":
                            self.vel.angular.z = -0.5
                            ctrl.steer(0.1)                                               # turn left
                        self._vel_lock.release()
            if self._vel_lock.acquire(False): 
                self.cmd_vel_pub.publish(self.vel)                                        # publish the twist to ROS                  
                self._vel_lock.release()
        else:
            ctrl.brake()                                                                      # stop      
            if self._vel_lock.acquire(False):         
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.vel)                                             # publish the twist to ROS 
                self._vel_lock.release()
        msg = String()
        msg.data = 'ang.z: %f lin.x: %f' % (self.vel.angular.z, self.vel.linear.x)
        self.publisher_.publish(msg)

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
