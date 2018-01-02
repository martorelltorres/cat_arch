#include <safety/RecoveryAction.h>
#include <robot_localization/SetPose.h>
#include <sensors/sensor_aggregator.h>

bool hasEnding(std::string const &fullString, std::string const &ending) {
  if (fullString.length() >= ending.length()) {
    return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}

SensorAggregator::SensorAggregator()
    : nhp_("~"), all_sensors_are_init_(false), north_(0), east_(0), initial_robot_pose_is_set_(false), tf_catched_(false) {

  // Setup the node name
  node_name_ = ros::this_node::getName();

  // Read parameters
  // Get sensor covariances
  nh_.param("/sensors/dvl_bottom_covariance", dvl_bottom_covariance_, 0.002);
  nh_.param("/sensors/dvl_water_covariance", dvl_water_covariance_, 0.02);
  nh_.param("/sensors/depth_covariance", depth_covariance_, 0.0);
  nh_.param("/sensors/gps_covariance", gps_covariance_, 0.0);
  nh_.param("/sensors/vo_covariance_xy", vo_covariance_xy_, 0.0);
  nh_.param("/sensors/vo_covariance_z", vo_covariance_z_, 0.0);

  // Disable safety?
  nh_.param("/sensors/enable_safety", enable_safety_, false);

  // Get sensor parameters
  nh_.param("sensors/dvl_max_v", dvl_max_v_, 2.0);
  nh_.param("sensors/gps_min_depth", gps_min_depth_, 0.3);
  nh_.param("sensors/max_num_errors", max_num_errors_, 5);
  nh_.param("sensors/gps_only_for_init", gps_only_for_init_, false);
  nh_.param("sensors/gps_init_filter_size", gps_init_filter_size_, 8);

  // Frames
  nh_.param("frames/map", map_frame_id_, std::string(""));
  nh_.param("frames/odom", odom_frame_id_, std::string(""));
  nh_.param("frames/base_link", robot_frame_id_, std::string(""));
  nh_.param("frames/sensors/origin_suffix", origin_suffix_, std::string(""));

  // Get set pose topic names
  nh_.param("sensors/set_pose_1", set_pose_1_, std::string("/ekf_odom/set_pose"));
  nh_.param("sensors/set_pose_2", set_pose_2_, std::string("/ekf_map/set_pose"));

  // Set emergency service
  if (enable_safety_)
  {
    ROS_INFO_STREAM_THROTTLE(1, "[" << node_name_ << "]: Waiting for safety service...");
    recovery_actions_ = nh_.serviceClient<safety::RecoveryAction>("/safety/recovery_action");
    bool is_available = recovery_actions_.waitForExistence(ros::Duration(10));
    if (!is_available) {
      ROS_ERROR_STREAM("[" << node_name_ << "]: RecoveryAction service is not available.");
    }
  }

  // Instantiate set of sensors
  dvl_   = new Sensor<DvlMsg,   TwistMsg>("dvl",   "/sensors/dvl_raw",   "/sensors/dvl"  );
  depth_ = new Sensor<PoseMsg,  PoseMsg> ("depth", "/sensors/depth_raw", "/sensors/depth_"+map_frame_id_, "/sensors/depth_"+odom_frame_id_);
  imu_   = new Sensor<ImuMsg,   ImuMsg>  ("imu",   "/sensors/imu_raw",   "/sensors/imu"  );
  gps_   = new Sensor<GpsMsg,   PoseMsg> ("gps",   "/sensors/gps_raw",   "/sensors/gps"  );
  modem_ = new Sensor<PoseMsg,  PoseMsg> ("modem", "/sensors/modem_raw", "/sensors/modem");
  vo_    = new Sensor<TwistMsg, TwistMsg>("vo",    "/sensors/vo_raw",    "/sensors/vo"   );

  modem_for_init_ = new Sensor<PoseMsg,  PoseMsg> ("modem_delayed", "/sensors/modem_delayed", "/sensors/modem_delayed_for_init");

  // Set up the set_pose publishers to set the poses of the ekf filters
  pose_1_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(set_pose_1_, 1);
  pose_2_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(set_pose_2_, 1);

  // Set up altitude publisher (from DVL)
  altitude_pub_ = nh_.advertise<sensor_msgs::Range>("/sensors/altitude", 1);

  // Set up callbacks
  dvl_->setCallback(boost::bind(&SensorAggregator::updateDvl, this, _1));
  depth_->setCallback(boost::bind(&SensorAggregator::updateDepth,this,_1));
  imu_->setCallback(boost::bind(&SensorAggregator::updateImu,this,_1));
  gps_->setCallback(boost::bind(&SensorAggregator::updateGps,this,_1));
  modem_->setCallback(boost::bind(&SensorAggregator::updateModem,this,_1));
  vo_->setCallback(boost::bind(&SensorAggregator::updateVo,this,_1));
  modem_for_init_->setCallback(boost::bind(&SensorAggregator::updateModemForInit,this,_1));
  // Create timer
  timer_ = nh_.createTimer(ros::Duration(0.5), &SensorAggregator::checkSensors, this);

  // Set up diagnostics
  updater_.setHardwareID("none");
  updater_.broadcast(0, "Starting SensorAggregator");
  updater_.add("SensorAggregator", this, &SensorAggregator::getCurrentState);
  updater_.update();
}

void SensorAggregator::updateDvl(const DvlMsg& msg) {
  // Take velocity ( bottom if possible )
  Eigen::Vector3d v(0.0, 0.0, 0.0);
  bool valid_data = false;
  double error = 0;
  /*  Error Velocity:
   *  A key quality control parameter that derives from the four
   *  beam geometry of an ADCP. Each pair of opposing beams provides one
   *  measurement of the vertical velocity and one component of the horizontal
   *  velocity, so there are actually two independent measurements of vertical
   *  velocity that can be compared. If the flow field is homogeneous, the
   *  difference between these vertical velocities will average to zero. To put
   *  the error velocity on a more intuitive footing, it is scaled to be
   *  comparable to the variance in the horizontal velocity. In a nutshell, the
   *  error velocity can be treated as an indication of the standard deviation
   *  of the horizontal velocity measurements.
   */
  double dvl_covariance;
  if (msg->bi_status == "A") { // && msg->bi_error > -32){
    v << msg->bi_x_axis, msg->bi_y_axis, msg->bi_z_axis;
    error = msg->bi_error;
    valid_data = true;
    dvl_covariance = dvl_bottom_covariance_;
  }
  else if (msg->wi_status == "A") { //&& msg->wi_error > -32) {
    v << msg->wi_x_axis, msg->wi_y_axis, msg->wi_z_axis;
    error = msg->wi_error;
    valid_data = true;
    dvl_covariance = dvl_water_covariance_;
  } else {
    dvl_->error("Bad data quality");
  }

  // Check speed requirements
  if (fabs(v[0]) < dvl_max_v_ &&
      fabs(v[1]) < dvl_max_v_ &&
      fabs(v[2]) < dvl_max_v_ / 4.0 &&
      valid_data) {
    // Create a new velocity message
    TwistMsg twist_msg;
    twist_msg.header.stamp = msg->header.stamp;
    twist_msg.header.frame_id = msg->header.frame_id;
    twist_msg.twist.twist.linear.x = v[0];
    twist_msg.twist.twist.linear.y = v[1];
    twist_msg.twist.twist.linear.z = v[2];
    // Check if error is negative. Don't believe it.
    if (error < 0) error = 0;
    // Only the number in the covariance matrix diagonal are used for the updates!
    twist_msg.twist.covariance[0]  = std::max(dvl_covariance, error*error);
    twist_msg.twist.covariance[7]  = std::max(dvl_covariance, error*error);
    twist_msg.twist.covariance[14] = std::max(dvl_covariance, error*error);
    if (all_sensors_are_init_)
      dvl_->publish(twist_msg);
    // check if altitude is valid
    if (msg->bi_status == "A" && msg->bd_time == 0.0) {
      sensor_msgs::Range alt_msg;
      alt_msg.header.stamp = msg->header.stamp;
      alt_msg.header.frame_id = msg->header.frame_id;
      alt_msg.range = msg->bd_range;
      //TODO this message type does not have covariance.
      if (all_sensors_are_init_)
        altitude_pub_.publish(alt_msg);
    }
  } else {
    dvl_->error("Invalid speed");
  }
}

void SensorAggregator::updateDepth(const PoseMsg& msg) {
  // Store last depth message
  {
    mutex::scoped_lock lock(mutex_depth_);
    depth_msg_ = msg;
  }

  // Change the frame ids
  PoseMsg map_msg = msg;
  PoseMsg odom_msg = msg;
  map_msg.header.frame_id  = msg.header.frame_id + "_" + map_frame_id_  + "_" + origin_suffix_;
  odom_msg.header.frame_id = msg.header.frame_id + "_" + odom_frame_id_ + "_" + origin_suffix_;

  double depth = msg.pose.pose.position.z;
  if (depth < -0.5) {
    depth_->error("Sensor probably not connected");
  } else {
    if (all_sensors_are_init_) {
      depth_->publish(map_msg, odom_msg);
    }
  }
}

void SensorAggregator::updateImu(const ImuMsg& msg) {
  // Store last imu message
  {
    mutex::scoped_lock lock(mutex_imu_);
    imu_msg_ = msg;
  }
  if (all_sensors_are_init_) {
    imu_->publish(msg);
  }
}

void SensorAggregator::updateGps(const GpsMsg& msg) {

  // Check depth to use the gps
  PoseMsg depth_msg;
  {
    mutex::scoped_lock lock(mutex_depth_);
    depth_msg = depth_msg_;
  }
  if (depth_msg.pose.pose.position.z > gps_min_depth_)
  {
    ROS_WARN_STREAM_THROTTLE(2, "[" << node_name_ << "]: Waiting for gps. Is the robot submerged?");
    return;
  }

  // Check pose message frame id
  std::string frame_id = msg->header.frame_id;
  if (!hasEnding(frame_id, "origin")) {
    ROS_WARN_STREAM("[" << node_name_ << "]: GPS sensor has incorrect frame_id: "
      << frame_id << ". Changing to gps_origin");
    frame_id = "gps_origin";
  }

  // Get the NED origin to convert lat/lon to x/y
  double ned_origin_lat, ned_origin_lon;
  bool s1, s2;
  s1 = nh_.getParamCached("/navigator/ned_origin_lat", ned_origin_lat);
  s2 = nh_.getParamCached("/navigator/ned_origin_lon", ned_origin_lon);

  if (s1 && s2) {
    // Set up current NED origin, in case it changes
    ned_ = new Ned(ned_origin_lat, ned_origin_lon, 0.0);

    // Take current lat, lon and convert it to NED
    double latitude  = msg->latitude;
    double longitude = msg->longitude;
    double north, east, depth;
    ned_->geodetic2Ned(latitude, longitude, 0.0, north, east, depth);

    // Security checks
    const double max_dist = 500.0;
    const double warn_dist = 100.0;
    double dist_to_ned_origin = sqrt(north*north + east*east);
    if (dist_to_ned_origin > max_dist) {
      ROS_ERROR_STREAM("[" << node_name_ << "]: Distance to NED origin too large: " << dist_to_ned_origin << "m (error at " << max_dist << "m).");
      return;
    } else if (dist_to_ned_origin > max_dist) {
      ROS_WARN_STREAM("[" << node_name_ << "]: Distance to NED origin is large: " << dist_to_ned_origin << " m (warning at " << warn_dist << "m).");
    }

    // Send the initial pose to the ekf
    if (!initial_robot_pose_is_set_) {
      // Filter the gps measures
      if (gps_samples_.size() < (uint)gps_init_filter_size_)
      {
        ROS_INFO_STREAM_THROTTLE(2, "[" << node_name_ << "]: Initializing gps: adding samples to gps filter.");
        gps_samples_.push_back(make_pair(north, east));
        return;
      }

      // Check the required variables to process gps
      if (!checkGpsRequiredVariables(msg->header.stamp.toSec()))
      {
        ROS_WARN_STREAM_THROTTLE(4, "[" << node_name_ << "]: Waiting for required gps variables...");
        return;
      }

      ROS_INFO_STREAM("[" << node_name_ << "]: Initializing gps: done.");

      // Compute the mean
      double north_mean = 0.0;
      double east_mean  = 0.0;
      for (size_t i=0; i<gps_samples_.size(); i++) {
        north_mean += gps_samples_[i].first;
        east_mean  += gps_samples_[i].second;
      }
      north_mean = north_mean / gps_samples_.size();
      east_mean  = east_mean  / gps_samples_.size();

      // Clear the vector
      gps_samples_.clear();

      // Start the publication of sensor messages
      initial_robot_pose_is_set_ = true;

      // Set the pose
      setPose(north_mean, east_mean, msg->position_covariance[0], msg->position_covariance[4], msg->header.stamp);
    }

    // Do a pose update
    if (all_sensors_are_init_ && !gps_only_for_init_) {

      // Create a new pose
      PoseMsg pose_msg;
      pose_msg.header.stamp = msg->header.stamp;
      pose_msg.header.frame_id = frame_id;
      pose_msg.pose.pose.position.x = north;
      pose_msg.pose.pose.position.y = east;
      // Do not use depth from geodetic2Ned as it is different from zero!
      pose_msg.pose.pose.position.z = 0.0;

      pose_msg.pose.covariance[0] = msg->position_covariance[0];
      pose_msg.pose.covariance[7] = msg->position_covariance[4];

      gps_->publish(pose_msg);
    }
  } else {
    ROS_ERROR_STREAM("The parameters: " <<
                     "\n\t\t* /navigator/ned_origin_lat" <<
                     "\n\t\t* /navigator/ned_origin_lon" <<
                     "\n\t\t  could not be correctly read from the parameter server. ");
    // Emergency surface
    safety::RecoveryAction srv;
    srv.request.error_level = srv.request.EMERGENCY_SURFACE;
    recovery_actions_.call(srv);
  }
}

void SensorAggregator::updateModem(const PoseMsg& msg) {
  PoseMsg pose_msg = msg;
  if (!hasEnding(msg.header.frame_id, "origin")) {
    ROS_WARN_STREAM("[" << node_name_ << "]: Modem sensor has incorrect frame_id: "
      << msg.header.frame_id << ". Changing to modem_origin");
    pose_msg.header.frame_id = "modem_origin";
  }

  // Set the pose
  if (!initial_robot_pose_is_set_)
  {
    ROS_INFO_STREAM("[" << node_name_ << "]: Setting the pose using the modem.");

    // Check the required variables to process gps
    if (!checkGpsRequiredVariables(pose_msg.header.stamp.toSec()))
    {
      ROS_WARN_STREAM_THROTTLE(2, "[" << node_name_ << "]: Waiting for required gps variables...");
      return;
    }

    initial_robot_pose_is_set_ = true;
    setPose(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, 1.0, 1.0, pose_msg.header.stamp);
  }

  if (all_sensors_are_init_) {
    PoseMsg depth_msg;
    {
      mutex::scoped_lock lock(mutex_depth_);
      depth_msg = depth_msg_;
    }
    if (depth_msg.pose.pose.position.z > 1.0)
      modem_->publish(pose_msg);
  }
}

void SensorAggregator::updateModemForInit(const PoseMsg& msg) {
  PoseMsg pose_msg = msg;

  // Set the pose
  if (!initial_robot_pose_is_set_)
  {
    ROS_INFO_STREAM("[" << node_name_ << "]: Setting the pose using the modem.");

    // Check the required variables to process gps
    if (!checkGpsRequiredVariables(pose_msg.header.stamp.toSec()))
    {
      ROS_WARN_STREAM_THROTTLE(2, "[" << node_name_ << "]: Waiting for required gps variables...");
      return;
    }

    initial_robot_pose_is_set_ = true;
    setPose(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, 1.0, 1.0, pose_msg.header.stamp);
  }
}

void SensorAggregator::updateVo(const TwistMsg& msg) {
  // Create a new velocity message
  TwistMsg twist_msg = msg;
  if (msg.twist.covariance[0] == 0) {
    ROS_INFO_STREAM_ONCE("[" << node_name_ << "]: Using provided covariance values for VO.");
    // Only the number in the covariance matrix diagonal are used for the updates!
    twist_msg.twist.covariance[0]  = vo_covariance_xy_;
    twist_msg.twist.covariance[7]  = vo_covariance_xy_;
    twist_msg.twist.covariance[14] = vo_covariance_z_;
  }
  if (all_sensors_are_init_)
    vo_->publish(twist_msg);
}

bool SensorAggregator::checkGpsRequiredVariables(double gps_stamp)
{
  // Get tf
  if (!tf_catched_) {
    // Catch static tfs
    try {
      tf_listener_.lookupTransform(robot_frame_id_,
                                   depth_->getName(),
                                   ros::Time(0),
                                   depth_tf_);
      tf_listener_.lookupTransform(robot_frame_id_,
                                   gps_->getName(),
                                   ros::Time(0),
                                   gps_tf_);
    } catch (tf::TransformException ex) {
      ROS_WARN_STREAM("[" << node_name_ << "]: " << ex.what());
      return false;
    }
    tf_catched_ = true;
  }

  if (imu_->init() && depth_->init() && tf_catched_) {

    mutex::scoped_lock lock1(mutex_imu_);
    mutex::scoped_lock lock2(mutex_depth_);

    double imu_stamp = imu_msg_.header.stamp.toSec();
    double depth_stamp = depth_msg_.header.stamp.toSec();
    if ( fabs( imu_stamp - gps_stamp ) > 1.0 ) {
      ROS_WARN_STREAM_THROTTLE(1, "[" << node_name_ << "]: Difference between gps stamp and imu stamp to big...");
      return false;
    }
    if ( fabs( depth_stamp - gps_stamp ) > 1.0 ) {
      ROS_WARN_STREAM_THROTTLE(1, "[" << node_name_ << "]: Difference between gps stamp and depth stamp to big...");
      return false;
    }

    return true;
  }

  return false;
}

void SensorAggregator::setPose(double north, double east, double north_cov, double east_cov, ros::Time gps_stamp) {

  geometry_msgs::PoseWithCovarianceStamped pose_msg;

  ImuMsg imu_msg;
  {
    mutex::scoped_lock lock(mutex_imu_);
    imu_msg = imu_msg_;
  }

  PoseMsg depth_msg;
  {
    mutex::scoped_lock lock(mutex_depth_);
    depth_msg = depth_msg_;
  }

  tf::Quaternion q(imu_msg.orientation.x,
                   imu_msg.orientation.y,
                   imu_msg.orientation.z,
                   imu_msg.orientation.w);
  tf::Transform imu_orientation(q);

  tf::Quaternion dummy_q(0, 0, 0, 1);

  tf::Vector3 gps_point(north, east, 0.0);
  tf::Transform gps_meas(dummy_q, gps_point);

  tf::Vector3 depth_point(depth_msg.pose.pose.position.x,
                          depth_msg.pose.pose.position.y,
                          depth_msg.pose.pose.position.z);
  tf::Transform depth_meas(dummy_q, depth_point);

  // Vehicle orientation = vehicle2imu * imu orientation
  tf::Transform robot_orientation = imu_orientation;

  // updated_measure = rot * measure * tf_sensor2sparus
  gps_meas   =  gps_meas  * robot_orientation *   gps_tf_.inverse();
  depth_meas = depth_meas * robot_orientation * depth_tf_.inverse();

  // Set pose
  pose_msg.pose.pose.position.x = gps_meas.getOrigin().x();
  pose_msg.pose.pose.position.y = gps_meas.getOrigin().y();
  pose_msg.pose.pose.position.z = depth_meas.getOrigin().z();
  pose_msg.pose.pose.orientation.x = robot_orientation.getRotation().x();
  pose_msg.pose.pose.orientation.y = robot_orientation.getRotation().y();
  pose_msg.pose.pose.orientation.z = robot_orientation.getRotation().z();
  pose_msg.pose.pose.orientation.w = robot_orientation.getRotation().w();
  // Set the covariances
  pose_msg.pose.covariance[0]  = north_cov;
  pose_msg.pose.covariance[7]  = east_cov;
  pose_msg.pose.covariance[14] = depth_msg.pose.covariance[14];
  pose_msg.pose.covariance[21] = imu_msg.orientation_covariance[0];
  pose_msg.pose.covariance[28] = imu_msg.orientation_covariance[4];
  pose_msg.pose.covariance[35] = imu_msg.orientation_covariance[8];

  // Set the stamp
  pose_msg.header.stamp = gps_stamp;

  // Send the message to the topic for ekf 1
  pose_msg.header.frame_id = odom_frame_id_;
  pose_1_pub_.publish(pose_msg);

  // Wait for tf of odom EKF to be ready
  bool ok = false;
  int count = 0;
  while (!ok && count < 30) {
    ok = tf_listener_.waitForTransform(odom_frame_id_, robot_frame_id_,
                                       ros::Time::now(), ros::Duration(1.0));
    ROS_INFO_STREAM_ONCE("[" << node_name_ << "]: Waiting for transform from "
      << odom_frame_id_ << " to "
      << robot_frame_id_);
    count++;
  }
  if (!ok) {
    ROS_WARN_STREAM("[" << node_name_ << "]: Transform from "
      << odom_frame_id_ << " to "
      << robot_frame_id_ << " was not found. Please check if ekf_odom is publishing.");
  }

  // Enable ekf_map filter
  enable_ekf_map_ = nh_.serviceClient<std_srvs::Empty>("/ekf_map/enable");
  while (!enable_ekf_map_.waitForExistence()) {
    ROS_INFO_STREAM_ONCE("[" << node_name_ << "]: Waiting for /ekf_map/enable service to be available.");
  }
  std_srvs::Empty srv1;
  enable_ekf_map_.call(srv1);

  // Send the message to the topic for ekf 2
  pose_msg.header.frame_id = map_frame_id_;
  pose_2_pub_.publish(pose_msg);

  // Enable the publication of nav_status
  enable_nav_status_ = nh_.serviceClient<sensors::EnableNavStatus>("/nav_status/enable");
  while (!enable_nav_status_.waitForExistence()) {
    ROS_INFO_STREAM_ONCE("[" << node_name_ << "]: Waiting for /nav_status/enable service to be available.");
  }
  sensors::EnableNavStatus srv2;
  srv2.request.enable = true;
  enable_nav_status_.call(srv2);

  // Log
  double r,p,y;
  robot_orientation.getBasis().getRPY(r, p, y);
  ROS_INFO_STREAM("[" << node_name_ << "]: /SetPose service called!"
    << "\n\t (X, Y, Z):  (" << pose_msg.pose.pose.position.x << ", "
                            << pose_msg.pose.pose.position.y << ", "
                            << pose_msg.pose.pose.position.z << ")"
    << "\n\t (R, P, Y):  (" << r << ", " << p << ", " << y << ")");
}

void SensorAggregator::checkSensors(const ros::TimerEvent& e) {
  if (!all_sensors_are_init_) {

    if (  dvl_->ready() && depth_->ready() &&
          imu_->ready() && modem_->ready() &&
           vo_->ready() &&   gps_->ready()) {
      if (!gps_->enabled()) {
        ROS_WARN_STREAM("[" << node_name_ << "]: GPS is disabled. A manual call to robot_localization/SetPose must be done in order to start the EKF");
        all_sensors_are_init_ = true;
      } else {
        if (initial_robot_pose_is_set_)
          all_sensors_are_init_ = true;
      }
    }
  } else {
    ROS_INFO_STREAM_ONCE("[" << node_name_ << "]: All sensors are READY!");
  }
  updater_.update();
}

void SensorAggregator::getCurrentState(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  // Check if all sensors are OK
  int num_errors = 0;
  if (!dvl_->ok()) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "DVL has encountered an error");
    dvl_->report(stat);
    num_errors += dvl_->getNumOfErrors();
  }
  if (!depth_->ok()) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Depth has encountered an error");
    depth_->report(stat);
    num_errors += depth_->getNumOfErrors();
  }
  if (!imu_->ok()) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "IMU has encountered an error");
    imu_->report(stat);
    num_errors += imu_->getNumOfErrors();

  }
  if (!gps_->ok()) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "GPS has encountered an error");
    gps_->report(stat);
    num_errors += gps_->getNumOfErrors();

  }
  if (!modem_->ok()) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "USBL has encountered an error");
    modem_->report(stat);
    num_errors += modem_->getNumOfErrors();
  }
  if (!vo_->ok()) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Visual Odometer has encountered an error");
    vo_->report(stat);
    num_errors += vo_->getNumOfErrors();
  }

  // Call recovery actions if there are too many errors
  if (num_errors == 0) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "All sensors work correctly");
  } else if (num_errors >= max_num_errors_ && enable_safety_) {
    // Emergency surface
    safety::RecoveryAction srv;
    srv.request.error_level = srv.request.EMERGENCY_SURFACE;
    recovery_actions_.call(srv);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_aggregator");
  SensorAggregator s;
  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();
  return 0;
}
