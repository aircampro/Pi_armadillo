/**
 * @file Turtlebot_simple_class.cpp
 *
 * @author aircampro
 *
 * @brief class for communcation with turtlebot on ROS and some simple movement tests to go up down a rectangular area
 *
 * ref :- https://github.com/s-kumada/DeliveryRobotNode2
 * 
 * License: BSD
 *   
 **/
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>     //!  https://demura.net/lecture/14011.html
#include <tf/transform_datatypes.h>

#include <ros/console.h>                                 // ログデバッグ出力用

#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// battery voltage
#include <sensor_msgs/BatteryState.h>                   //バッテリーステータス     TB3
#include <std_msgs/Int16MultiArray.h>                   //バッテリーステータス     メガローバ

// inverse kinetics safe asin acos and atan2 functions as macros
#define IKPI_2  ((double)1.57079632679490f)
// allows asin and acos to exceed 1. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((double)1e-7f)
#endif
// used to check input to atan2 for degenerate cases. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((double)1e-7f)
#endif
// safe check for asin
#define IKsin(ret,a1) do{ if (!IsaNan(a1) && ((a1 > -1-IKFAST_SINCOS_THRESH) && (a1 < 1+IKFAST_SINCOS_THRESH))) {  ret = asin(a1); } else if (a1<=-1) { ret = -IKPI_2; } else if (a1>=1) { ret = IKPI_2; } } while(0)
// safe check for acos
#define IKcos(ret,a1) do{ if (!IsaNan(a1) && ((a1 > -1-IKFAST_SINCOS_THRESH) && (a1 < 1+IKFAST_SINCOS_THRESH))) {  ret = acos(a1); } else if (a1<=-1) { ret = PI; } else if (a1>=1) { ret = 0.0f; } } while(0)
// safe check for atan2
#define IKatan2(ret,a1,a2)  do{ if ((!IsaNan(a1) && !IsaNan(a2)) && (((a1 >= IKFAST_ATAN2_MAGTHRESH) || (a2 >= IKFAST_ATAN2_MAGTHRESH)) || ((-a1 <= -IKFAST_ATAN2_MAGTHRESH) || (-a2 <= -IKFAST_ATAN2_MAGTHRESH)))){  ret = atan2(a1,a2); } else if (IsaNan(a1)) { ret = IKPI_2; } else if (IsaNan(a2)) { ret = 0.0f; } } while(0)

#ifndef M_PI
#define M_PI (double)(4.0f * atan(1.0f))             
#endif
#define DEG2RAD(x) ((x)*M_PI/180.0f)  
#define RAD2DEG(x) ((x)*180.0f/M_PI)  
#define RADPSEC_FROM_METREPSEC(ms) { (16.5289f*ms) }                            // Convert meters/sec into radians/sec

// ID,TYPE
#define     DEFAULT_ROBOT_ID    "turtlebot_01"
#define     DEFAULT_ROBOT_TYPE  "turtlebot"
#define     DEFAULT_TURN_SPEED  0.4f

#define     DEFAULT_VELOCITY_TOPIC  "cmd_vel"

// メガローバ 電圧係数
#define     MR_VOLTAGE_FACTOR   137.8

#define     ROS_RATE_10HZ       10.0f
#define     ROS_RATE_20HZ       20.0f
#define     ROS_RATE_30HZ       30.0f
#define     ROS_RATE_50HZ       50.0f

#define     ROS_TIME_0S         0
#define     ROS_TIME_50MS       0.05f
#define     ROS_TIME_500MS      0.5f
#define     ROS_TIME_1S         1.0f
#define     ROS_TIME_3S         3.0f
#define     ROS_TIME_5S         5.0f

#define     ANGLE_OF_0_DEGREES      0
#define     ANGLE_OF_3_DEGREES      3.0f
#define     ANGLE_OF_360_DEGREES    360.0f
#define     ANGLE_OF_180_DEGREES    180.0f
#define     ANGLE_OF_90_DEGREES     90.0f

#define     ROS_QUEUE_SIZE_5    5
#define		ROS_QUEUE_SIZE_10	10
#define		ROS_QUEUE_SIZE_100	100

#define NUM_OF_PATHS 10                                                 // number of paths in the rectangle
#define STEP_DIST 1                                                     // distance of one step
#define PATH_LEN 200                                                    // one upward path length
#define NUM_OF_RPEATS 5                                                 // num of times it scans the area 

class RobotDriver
{
private:
  //! The node handle we'll be using                                    // 使用するノードハンドル
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands    //コマンドを発行するために "cmd_vel"トピックに公開します。
  ros::Publisher cmd_vel_pub_;
  //! initial pose
  ros::Publisher pub_initial;
  //! We will be listening ot TF transforms as well for drive forward
  tf::TransformListener listener_;
  //! for odom - using separate handles to allow for multithreading
  tf::TransformListener tfl;
  //! for map query
  tf::TransformListener tf2;
  
  //! publisher to set goal
  ros::Publisher pub_goal;                                               // 目標地点地点の送信用パブリッシャ

  //! subscribe to read the battery voltage
  ros::Subscriber sub_battery_state_recv;                                // バッテリー情報受信用サブスクライバ
  //! subscribe to read the amcl pose
  ros::Subscriber sub_amclpose_recv;                                     // amcl_pose受信受信用サブスクライバ
	
  bool move_forward_state_;
  bool move_side_state_;
  bool move_turn_state_;
	
  std::string entityId_;                                                 // ロボットのユニークID
  std::string _entity_type;
  std::string velocityTopic_;                                            // velocityトピック名
  double _navigation_turn_speed;                                         // navigation turn speed
  std::vector<double> _init_pose;                                        // initial pose
  float _volt_sts;                                                       // battery volts
  double _g_covariance[36];                                              // 共分散値
	
  geometry_msgs::PoseWithCovarianceStamped _initial_pose; 

  enum Action_States_e
	{
		UP_DOWN_PATH,
		TURN_TO_NEXT,
		RETURN_TO_HOME,
	};	
public:
  //! ROS node initialization       //  ROSノードの初期化
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;

    move_forward_state_ = false;
    move_side_state_ = false;
    move_turn_state_ = false;

  }

  RobotDriver(ros::NodeHandle &nh, ros::NodeHandle &privateNode)
  {
    nh_ = nh;

    move_forward_state_ = false;
    move_side_state_ = false;
    move_turn_state_ = false;

    memset( &_g_covariance, 0, sizeof(_g_covariance)); 
    _g_covariance[0] = 0.25f;
    _g_covariance[7] = 0.25f;
    _g_covariance[35] = 0.06853891945200942f;
		
    if (privateNode.getParam("entity_id", entityId_)){
        ROS_INFO("RobotDriver entity_id:%s", entityId_.c_str());
    }else{
        entityId_ = DEFAULT_ROBOT_ID;
    }

    if (privateNode.getParam("velocity_topic", velocityTopic_)){ROS_INFO("RobotDriver velocity_topic:%s", velocityTopic_.c_str());
    }else{
        velocityTopic_ = DEFAULT_VELOCITY_TOPIC;
    }

    if (privateNode.getParam("initial_turn_speed", _navigation_turn_speed)){
        ROS_INFO("initial_turn_speed (%lf)", _navigation_turn_speed);
    }else{
        _navigation_turn_speed = DEFAULT_TURN_SPEED;                                          // rad
    }
	
    if (privateNode.getParam("initial_pose", _init_pose)){
        ROS_INFO("initial_pose(%lf,%lf,%lf)", _init_pose[0], _init_pose[1], _init_pose[2]);
    }else{
        ROS_ERROR("not initial pose");
        return(-1);
    }

    if (privateNode.getParam("entity_type", _entity_type))
    {
        ROS_INFO("entity_type (%s)", _entity_type.c_str());
    }
    else
    {
        _entity_type = DEFAULT_ROBOT_TYPE;
    }
		
    // set up the publisher for the cmd_vel topic    //cmd_velトピックの発行者を設定します。
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>( entityId_ + "/" + velocityTopic_, 1);
	
	// set up the publisher for the initial pose topic
    pub_initial = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/" + _entityId + "/initialpose", ROS_QUEUE_SIZE_5, true);

	// set up the publisher for the pose goal topic	
	pub_goal = node.advertise<geometry_msgs::PoseStamped>("/" + _entityId + "/move_base_simple/goal", ROS_QUEUE_SIZE_5, false);

    // subscribe to get battery volts
    if( _entity_type == DEFAULT_ROBOT_TYPE ){
        sub_battery_state_recv = node.subscribe("/" + _entityId + "/battery_state", ROS_QUEUE_SIZE_10, &RobotDriver::batteryStateRecv, this);
    }else{
        sub_battery_state_recv = node.subscribe("/" + _entityId + "/rover_sensor", ROS_QUEUE_SIZE_10, &RobotDriver::batteryStateRecvMr, this);
    }
	
	// amcl_pose受信
    sub_amclpose_recv = node.subscribe("/" + _entityId + "/amcl_pose", ROS_QUEUE_SIZE_10, &RobotDriver::amclPoseRecv, this);
  }

  void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat)
  {
        tf::Quaternion quat;
        quaternionMsgToTF(geometry_quat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
  }

  geometry_msgs::Quaternion GetQuaternionMsg(double roll, double pitch, double yaw )
  {
        geometry_msgs::Quaternion q;
        tf::Quaternion quat =tf::createQuaternionFromRPY(roll,pitch,yaw);
        quaternionTFToMsg( quat, q);

        return(q); 
  }
  
  //! Drive forward a specified distance based on odometry information  // 走行距離情報に基づいて指定された距離を前進する
  // 並進   ※バックは不可 
  // distance： >0  走行距離
  bool driveForwardOdom(double distance)
  {
    //wait for the listener to get the first message    // リスナーが最初のメッセージを受け取るのを待つ

    try{
        ROS_DEBUG("--- waitForTransform --- Before  ");
        ros::Time now = ros::Time::now();
        listener_.waitForTransform(entityId_ + "_base_link", entityId_ + "_odom", now, ros::Duration(3.0));   
        ROS_DEBUG("--- waitForTransform --- After");
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("---  catch msg  --- waitForTransform");

      ROS_ERROR("%s",ex.what());
      return(false);
    }

    //we will record transforms here    // ここで変換を記録します
    tf::StampedTransform start_transform;       // 開始位置
    tf::StampedTransform current_transform;     // 現在位置
    //record the starting transform from the odometry to the base frame     // オドメトリからベースフレームへの開始変換を記録する
    ROS_DEBUG("--- lookupTransform --- Before");

    try{
        listener_.lookupTransform( entityId_ + "_base_link", entityId_ + "_odom", ros::Time(ROS_TIME_0S), start_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("---  catch msg --- lookupTransform");

      ROS_ERROR("%s",ex.what());
      return(false);
    }

    ROS_DEBUG("--- lookupTransform --- After");

    //we will be sending commands of type "twist"   //  "twist"タイプのコマンドを送信します
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s     // コマンドは0.25m/sで前進することです（直進速度）
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;

    ros::Rate rate(ROS_RATE_50HZ);   //  50Hz（1秒間に50回、この場合0.02秒間隔）
    bool done = false;

    ROS_DEBUG("start");
    ROS_DEBUG("x(%f) y(%f) z(%f) w(%f)",start_transform.getOrigin().x(),start_transform.getOrigin().y(),
    start_transform.getOrigin().z(), start_transform.getOrigin().w());
    ROS_DEBUG("Advance"); // 前進

    while (!done && nh_.ok())   //doneまで、またはCtrl+Cが押されるまで
    {
      //send the drive command      //  ドライブコマンドを送信する
      cmd_vel_pub_.publish(base_cmd);   // パブリッシュ
      rate.sleep();                     // 指定Hz間隔にするようにスリープ
      //get the current transform   //  現在の変換を取得する
      try
      {
        listener_.lookupTransform(entityId_ + "_base_link", entityId_ + "_odom", ros::Time(ROS_TIME_0S), current_transform);     // ros::Time(0) = 最新の情報を読み出す
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled  // 走行距離を計算
      tf::Transform relative_transform = start_transform.inverse() * current_transform;              // inverse()=この変換の逆を返します。

      double dist_moved = relative_transform.getOrigin().length();  // getOrigin()=Vector3型を返す  length()=ベクトルの長さを返します。
//      ROS_DEBUG("x(%f) y(%f) z(%f) w(%f)",current_transform.getOrigin().x(),current_transform.getOrigin().y(),
//      current_transform.getOrigin().z(), current_transform.getOrigin().w());
      ROS_DEBUG("distance = %f dist_moved= %f",distance, dist_moved );

      if(dist_moved > distance) done = true;
    }

    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    cmd_vel_pub_.publish(base_cmd);

    if (done) return true;
    return false;
  }

  //  bool clockwise：true=右回転   false=左回転
  //  double radians：回転角度(ラジアン)    DEG2RAD(x) x=角度
  bool turnOdom(bool clockwise, double radians, double angularSpeed = 0.4)
  {
    double radians_before = radians;

    //wait for the listener to get the first message    // リスナーが最初のメッセージを受け取るのを待つ
    ros::Time now = ros::Time::now();
    try{
        listener_.waitForTransform( entityId_ + "_base_link", entityId_ + "_odom", now, ros::Duration(ROS_TIME_3S));   // タイムアウト1秒
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("---  catch msg  --- waitForTransform");

      ROS_ERROR("%s",ex.what());
      return(false);
    }

    //we will record transforms here        // ここで変換を記録します
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    double yaw_before = 0;
    double yaw_current = 0;
    double yaw_total = 0;

    //record the starting transform from the odometry to the base frame // オドメトリからベースフレームへの開始変換を記録する

    try{
        listener_.lookupTransform(entityId_ + "_base_link", entityId_ + "_odom", ros::Time(ROS_TIME_0S), start_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("---  catch msg --- lookupTransform");

      ROS_ERROR("%s",ex.what());
      return(false);
    }

    //we will be sending commands of type "twist"   // "twist"タイプのコマンドを送信します
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s     // コマンドは0.4 rad / sで回転するようになります（回転速度）
    base_cmd.linear.x = base_cmd.linear.y = 0.0f;
    base_cmd.angular.z = angularSpeed;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z; // 回転方向を決める（＋は左回転、－は右回転）

    double turn_threshold = fabs(base_cmd.angular.z) / 5.0f;// 回転量のしきい値 0.2s時の最大回転rad

    //the axis we want to be rotating by            //  回転させたい軸
    tf::Vector3 desired_turn_axis(0,0,1);           //  z軸を指定
    if (!clockwise) desired_turn_axis = -desired_turn_axis;

    ros::Rate rate(ROS_RATE_50HZ);   //  50Hz（1秒間に50回、この場合0.02秒間隔）
    bool done = false;

    double angle_turned_before = 0;
    double angle_turned_total = 0;

    while (!done && nh_.ok())   //doneまで、またはCtrl+Cが押されるまで
    {
      //send the drive command      // ドライブコマンドを送信する
      cmd_vel_pub_.publish(base_cmd);   // パブリッシュ
      rate.sleep();                     // 指定Hz間隔にするようにスリープ
      //get the current transform   //  現在の変換を取得する
      try
      {
        listener_.lookupTransform(entityId_ + "_base_link", entityId_ + "_odom", ros::Time(ROS_TIME_0S), current_transform);

        tf::Matrix3x3 m = current_transform.getBasis();
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        yaw_current = yaw; // 現在のyawを取得   

      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }

      if ( fabs(yaw_current-yaw_before) < 1.0e-2) continue;       // fabs=絶対値

      double turn_val;

      if( clockwise ){
          // 右回転     
          if(yaw_current >= yaw_before){
             turn_val = yaw_current - yaw_before;
             ROS_DEBUG(">= yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
          }else{  
             turn_val = (M_PI - yaw_before) + (M_PI + yaw_current);
             ROS_DEBUG("< yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
          }
      }else{
          // 左回転           
          if(yaw_before >= yaw_current){
             turn_val = yaw_before - yaw_current;       
             ROS_DEBUG(">= yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
          }else{  
             turn_val = (M_PI - yaw_current) + (M_PI + yaw_before);
             ROS_DEBUG("< yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
          }
      }
      if(turn_val <= turn_threshold){
          yaw_total += turn_val; // 加算
      }else{
          ROS_WARN("turn_threshold(%f) turn_val(%f)",turn_threshold, turn_val);
      }

      yaw_before = yaw_current; // 前回値保持

      if (yaw_total > radians) done = true;

    }

    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    cmd_vel_pub_.publish(base_cmd);

    if (done) return true;
    return false;
  }

  bool moveForwardState()
  { 
    return(move_forward_state_);
  }
  bool moveSideState()
  { 
    return(move_side_state_);
  }
  bool moveTurnState()
  { 
    return(move_turn_state_);
  }
  float getBatterVoltage()
  { 
    return(_volt_sts);
  }
  
  bool stopOdom()
  { 
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.0;
    cmd_vel_pub_.publish(base_cmd);
    move_forward_state_ = false;
    move_side_state_ = false;	
    move_turn_state_ = false;
	return false
  }
  bool moveForward(double linearSpeed)
  { 
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = linearSpeed;
    cmd_vel_pub_.publish(base_cmd);

    move_forward_state_ = true;
	return false
  }
  bool moveForwardSide(double x, double y)
  { 
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.y = y;
	base_cmd.angular.z = 0;
    base_cmd.linear.x = x;
    cmd_vel_pub_.publish(base_cmd);

    move_forward_state_ = true;
	return false
  }
  bool moveAllDirections(double x, double y, double z)
  { 
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.y = y;
	base_cmd.angular.z = z;
    base_cmd.linear.x = x;
    cmd_vel_pub_.publish(base_cmd);

    move_forward_state_ = true;
	return false
  }
  bool moveSideways(double linearSpeed)
  { 
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.angular.z = 0;
    base_cmd.linear.y = linearSpeed;
    cmd_vel_pub_.publish(base_cmd);

    move_side_state_ = true;
	return false
  }
  bool moveTurn(bool clockwise, double angularSpeed)
  { 
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = angularSpeed;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z; // 回転方向を決める（＋は左回転、－は右回転）
    cmd_vel_pub_.publish(base_cmd);
    move_turn_state_ = true;
	return false
  }
  void initialPoseSend(const float x, const float y, const float yaw)
  {
    _initial_pose.header.frame_id = "map";
    _initial_pose.pose.pose.position.x = x;
    _initial_pose.pose.pose.position.y = y;
    _initial_pose.pose.pose.orientation = GetQuaternionMsg( 0, 0, yaw);

    memset(&_initial_pose.pose.covariance[0], 0, sizeof(_initial_pose.pose.covariance));//共分散行列（散らばり具合を表す指標）
    _initial_pose.pose.covariance[0] = 0.25;
    _initial_pose.pose.covariance[7] = 0.25;
    _initial_pose.pose.covariance[35] = 0.06853891945200942;
    pub_initial.publish(_initial_pose);
	return;
  }
  void turn_to_angle(double turn_speed, double angle_degree) 
  {
    double yaw;

    try
    {
        // 現在のロボットの向きを取得
        tf::StampedTransform trans;
        // Durationは実機:0.5, シミュレータ:5.0
        tfl.waitForTransform(_entityId + "/base_link", _entityId + "/odom", ros::Time(ROS_TIME_0S), ros::Duration(ROS_TIME_5S));
        tfl.lookupTransform(_entityId + "/base_link", _entityId + "/odom", ros::Time(ROS_TIME_0S), trans);// base_linkから見たodom、(ロボットの向き)
        yaw = tf::getYaw(trans.getRotation());                                                            // 四元数からyaw角を取得
    }
    catch(tf::TransformException &e)
    {
        ROS_ERROR("turn360() err(%s)", e.what());
        return;
    }

    double turn_threshold = turn_speed / 5;                                                               // 回転量のしきい値 0.2s時の最大回転rad

    double yaw_before = 0;
    double yaw_current = yaw;
    double yaw_total = 0;
    double turn_val;

    ROS_INFO("turn (%f rad/s) start ---------->",turn_speed);

	ros::Rate rate(ROS_RATE_10HZ);

    while(ros::ok())
    {
        moveTurn(true, turn_speed);                                                                        // 旋回させる(右回転)

        rate.sleep();
        ros::spinOnce();
 
        try
        {
            // 現在のロボットの向きを取得
            tf::StampedTransform trans;
            ros::Time now = ros::Time::now();
            tfl.waitForTransform(_entityId + "/base_link", _entityId + "/odom", now, ros::Duration(ROS_TIME_1S));
            tfl.lookupTransform(_entityId + "/base_link", _entityId + "/odom", now, trans);
            yaw_current = tf::getYaw(trans.getRotation());                                               // 四元数からyaw角を取得
        }
        catch(tf::TransformException &e)
        {
            ROS_ERROR("turn360() err(%s)", e.what());
            return;
        }

        if ( fabs(yaw_current-yaw_before) < 1.0e-2) continue;                                            // 小さすぎる値はスルー // fabs=絶対値

        // 右回転     
        if(yaw_current >= yaw_before){
			turn_val = yaw_current - yaw_before;
            ROS_DEBUG(">= yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
        }else{  
            turn_val = (M_PI - yaw_before) + (M_PI + yaw_current);
            ROS_DEBUG("< yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
        }

        if(turn_val <= turn_threshold){
            yaw_total += turn_val;                                                                             // 加算
        }else{
            ROS_WARN("turn_threshold(%f) turn_val(%f)",turn_threshold, turn_val);
        }

        yaw_before = yaw_current;                                                                            // 前回値保持

        if (yaw_total >= DEG2RAD(angle_degree)){
            break;
        }
    }

    stopOdom();                                                                                              // いったん停止

    ROS_INFO("turn end <-------------");
        
    return;
  }
  geometry_msgs::PoseStamped makePoseStamped(double x, double y, double yaw)
  {
    geometry_msgs::PoseStamped  posePoseStamped;

    posePoseStamped.header.frame_id = "map";
    posePoseStamped.pose.position.x = x;
    posePoseStamped.pose.position.y = y;
    posePoseStamped.pose.orientation = GetQuaternionMsg( 0, 0, yaw);

    return( posePoseStamped );
  }
  void simpleGoalSend(double x, double y, double yaw)
  {
    geometry_msgs::PoseStamped way_goal = makePoseStamped( x, y, yaw);            
    pub_goal.publish(way_goal);

    return;
  }
  void simpleGoalSendOdom(double x, double y)
  {
    double yaw;

    try
    {
        // 現在のロボットの向きを取得
        tf::StampedTransform trans;
        // Durationは実機:0.5, シミュレータ:5.0
        tfl.waitForTransform(_entityId + "/base_link", _entityId + "/odom", ros::Time(ROS_TIME_0S), ros::Duration(ROS_TIME_5S));
        tfl.lookupTransform(_entityId + "/base_link", _entityId + "/odom", ros::Time(ROS_TIME_0S), trans);// base_linkから見たodom、(ロボットの向き)
        yaw = tf::getYaw(trans.getRotation());                                                            // 四元数からyaw角を取得
    }
    catch(tf::TransformException &e)
    {
        ROS_ERROR("turn360() err(%s)", e.what());
        return;
    }

    geometry_msgs::PoseStamped way_goal = makePoseStamped( x, y, yaw);            
    pub_goal.publish(way_goal);

    return;
  }
  bool getCurrentCoordinatesFromMap( double& cur_x, double& cur_y, double& cur_yaw )
  {
    bool ret_sts = true;
    try
    {
        tf::StampedTransform trans;
        tf2.waitForTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), ros::Duration(ROS_TIME_500MS));
        tf2.lookupTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), trans);  // mapから見たbase_link、(world座標のロボットの位置)
        cur_x = trans.getOrigin().x();                // Ｘ座標
        cur_y = trans.getOrigin().y();                // Ｙ座標
        cur_yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
        ret_sts = false;
    }
    catch(tf::TransformException &ex)
    {
        ROS_WARN("currentCoordinates() err(%s)", ex.what());
    }

    return( ret_sts );
  }
  bool addCoordGoalSendMap(double x, double y, double yaw)
  {
    double c_x,x_y,c_yaw;
	bool ret_sts = true;
    if (getCurrentCoordinatesFromMap(&c_x, &c_y, &c_yaw)==false) {
        geometry_msgs::PoseStamped way_goal = makePoseStamped( c_x+x, c_y+y, c_yaw+yaw);            
        pub_goal.publish(way_goal);
        ret_sts = false;
	}

    return( ret_sts );
  }
  bool getTurnAngle(const double rad_base, const double rad , double &turn_rad)
  {
        turn_rad = 0;
        bool blsts = true;
        double current_rad = rad_base + M_PI;
        double goal_rad    = rad      + M_PI;

        if( current_rad >= goal_rad ){
            if( (current_rad - goal_rad) <= M_PI ){
                blsts = true;   
                turn_rad = current_rad - goal_rad;
            }else{
                blsts = false;  
                turn_rad = 2*M_PI - (current_rad - goal_rad);
            }
        }else{
            if( (goal_rad - current_rad) <= M_PI ){
                turn_rad = goal_rad - current_rad;
                blsts = false; 
            }else{
                turn_rad = 2*M_PI - (goal_rad - current_rad);
                blsts = true;   
            }
        }

        return( blsts );
  }
  //--------------------------------------------------------------------------
  //  wayポイント方向に向きを変える
  //--------------------------------------------------------------------------
  double turnTowardsGoal(double dest_x, double dest_y)
  {
        double x, y, yaw;
        try
        {
            // 現在のロボットの向きを取得
            tf::StampedTransform trans;
            tf2.waitForTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), ros::Duration(ROS_TIME_500MS));
            tf2.lookupTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), trans);
            x = trans.getOrigin().x();               
            y = trans.getOrigin().y();                
            yaw = tf::getYaw(trans.getRotation());    
        }
        catch(tf::TransformException &e)
        {
            ROS_WARN("turnTowardsGoal() err(%s)", e.what());
            return(0.0);
        }

        // goalに向かう向きを取得  
        // safe yaw_way = atan2((dest_y - y),(dest_x - x);		
        double yaw_way; 
        IKatan2(yaw_way,(dest_y - y),(dest_x - x));

        // 旋回角度を取得
        double turn_rad = 0; //旋回角度
        bool turn_direction = getTurnAngle( yaw, yaw_way, turn_rad);

        // 旋回させる
        ros::Rate rate(ROS_RATE_30HZ);   // 30Hz処理

        for(;;)
        {
            // 旋回させる
            moveTurn(turn_direction, _navigation_turn_speed);  
 
            rate.sleep();
            ros::spinOnce();

            try
            {
                // 現在のロボットの向きを取得
                tf::StampedTransform trans;
                tf2.waitForTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), ros::Duration(ROS_TIME_500MS));
                tf2.lookupTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), trans);  // mapから見たbase_link、(world座標のロボットの位置)
                x = trans.getOrigin().x();                // Ｘ座標
                y = trans.getOrigin().y();                // Ｙ座標
                yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
            }
            catch(tf::TransformException &e)
            {
                ROS_WARN("turnTowardsGoal() err(%s)", e.what());
                continue;
            }

            // goalに向かう向きを取得 
            // safe yaw_way = atan2((dest_y - y),(dest_x - x);			
            IKatan2(yaw_way,(dest_y - y),(dest_x - x));
		
            // 旋回角度を取得
            turn_rad = ANGLE_OF_0_DEGREES; //旋回角度
            bool turn_direction_now = getTurnAngle( yaw, yaw_way, turn_rad);

            if(turn_direction_now != turn_direction){
                ROS_INFO("turn_direction reverse");
                turn_direction = turn_direction_now;
            } 

            // WP方向までの角度差が3度以下か判定する
            if(RAD2DEG(turn_rad) <= ANGLE_OF_3_DEGREES){
                break;
            } 
        } 
        stopOdom();  // いったん停止

        return( yaw_way );
  }  
  //------------------------------------------------------------------------------
  //  バッテリステータス受信  30Hz
  //------------------------------------------------------------------------------
  void batteryStateRecv(const sensor_msgs::BatteryState msg)
  {
    _volt_sts = msg.voltage;
        
    return;
  }

  //------------------------------------------------------------------------------
  //  バッテリステータス受信  メガローバ  90Hz
  //------------------------------------------------------------------------------
  void batteryStateRecvMr(const std_msgs::Int16MultiArray msg)
  {
    if(msg.data.size() >= 10){
        _volt_sts = msg.data[9] / MR_VOLTAGE_FACTOR;
    }
        
    return;
  }
  //------------------------------------------------------------------------------
  //  amcl_pose受信
  //------------------------------------------------------------------------------
  void amclPoseRecv(const geometry_msgs::PoseWithCovarianceStamped msg)
  {
    try
    {
       memcpy( &_g_covariance, &msg.pose.covariance, sizeof(_g_covariance));//共分散
    }
    catch(const std::exception &e)
    {
       ROS_WARN("amcl_pose conversion errer[%s]", e.what());
    }
        
    return;
  }
	
};

/******************************************************************************/
/* main                                                                       */
/******************************************************************************/
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "delivery_robot_node");

    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    RobotDriver driver(node, privateNode);

    float batt_voltage = getBatterVoltage();
    std::cout << "battery volts : " << batt_voltage << std::endl;
	
    double turn_speed = driver._navigation_turn_speed;
    driver.initialPoseSend(driver._init_pose[0], driver._init_pose[1], driver._init_pose[2]);                 //initialPose送信

    int states = 0;
    int cnt = 0;
	int no_of_rpeats = 0;
	double distance = PATH_LEN;
	double angle_degree = 180;
    bool is_running = true;

    while (is_running) {	
        switch(states) {
            case Action_States_e::UP_DOWN_PATH:
            {
                // drive forward stop and turn back
	            distance = PATH_LEN;
	            angle_degree = 180;
                driver.driveForwardOdom(distance);
                driver.stopOdom();
	            driver.turn_to_angle(turn_speed, angle_degree);
                driver.driveForwardOdom(distance);
                driver.stopOdom();
			    states = Action_States_e::TURN_TO_NEXT;
            }
            break;

            case Action_States_e::TURN_TO_NEXT:
            {
                // now turn 90 degree, walk a little then turn a further 90	(both anticlockwise)
	            angle_degree = 90;
                driver.turnOdom(false, DEG2RAD(angle_degree));	
                distance = STEP_DIST;
                driver.driveForwardOdom(distance);
                driver.stopOdom();
                driver.turnOdom(false, DEG2RAD(angle_degree));
                ++cnt;
                if (cnt >= NUM_OF_PATHS) {
                  states = Action_States_e::RETURN_TO_HOME;
                } else {
                  states = Action_States_e::UP_DOWN_PATH;
                }
		    }
            break;
		
            case Action_States_e::RETURN_TO_HOME:
            {
	            angle_degree = 90;
                driver.turnOdom(true, DEG2RAD(angle_degree));	
                distance = NUM_OF_PATHS*STEP_DIST;
                driver.driveForwardOdom(distance);
                driver.stopOdom();			
				++no_of_rpeats;
                if (no_of_rpeats >= NUM_OF_RPEATS) {
                   is_running = false;
                } else {
                   states = Action_States_e::UP_DOWN_PATH;
                }					
            }
            break;
        }
    }	

    double dest_x = 0.7f;
	double dest_y = 0.6f;
    double yaw_out = driver.turnTowardsGoal(dest_x, dest_y);
    std::cout << " yaw out at turn x=0.7 y=0.6 " << yaw_out << std::endl;
	driver.turn_to_angle(turn_speed, yaw_out);
    driver.driveForwardOdom(5.0f);
    driver.stopOdom();
				
    return(0);
}

