#include "squirrel_person_tracker/squirrel_person_tracker.h"

SquirrelTracker::SquirrelTracker(ros::NodeHandle& pnh_) :
    pcl_msg(new pcl::PointCloud<pcl::PointXYZ>) {
  if (!pnh_.getParam("camera_optical_frame_id", frame_id)) {
    frame_id = "camera_depth_optical_frame";
    ROS_INFO("camera_optical_frame_id is not set. Default value: %s", frame_id.c_str());
  }

  if (!pnh_.getParam("tf_plane", tf_plane_)) {
    tf_plane_ = true;
    ROS_INFO("tf_plane is not set. Default value: true");
  }

  if (!pnh_.getParam("pub_filtered_pc", pub_filtered_pc_)) {
    pub_filtered_pc_ = false;
    ROS_INFO("pub_filtered_pc is not set. Default value: false");
  }

  if (!pnh_.getParam("publish_wuser_skeleton", publish_wuser_skeleton_)) {
    publish_wuser_skeleton_ = true;
    ROS_INFO("pub_filtered_pc is not set. Default value: true");
  }

  if (!pnh_.getParam("track_wave_user", track_wave_user_)) {
    track_wave_user_ = true;
    ROS_INFO("pub_filtered_pc is not set. Default value: true");
  }

  ostatus = openni::STATUS_OK;
  nstatus = nite::STATUS_OK;
  if (pub_filtered_pc_) {
    pubPC = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("filtered_cloud", 1);
    pcl_msg->height = 1;
  }
  pubPP = pnh_.advertise<geometry_msgs::PointStamped>("pointing_pose", 10);
  pubPSA = pnh_.advertise<geometry_msgs::PoseArray>("detected_pose", 10);
  pubState = pnh_.advertise<squirrel_person_tracker_msgs::State>("tracking_state", 10);
  pubPHH = pnh_.advertise<squirrel_person_tracker_msgs::HeadHandPoints>("head_hand_points", 10);
  pubSkelVec = pnh_.advertise<squirrel_person_tracker_msgs::SkeletonVector>("user_information", 10);
  wavingUserID = 0;
  ISWAVEDETECTED = false;
  SWITCHSKELTRACKON = true;
  ISWAVEUSERVISBLE = false;
  publishedState.state = squirrel_person_tracker_msgs::State::NO_USER;
  beginUnvis = 0;
  durationUnvis = 3;
  transform.setRotation(tf::Quaternion(0, 0, 0, 1));
  this->initUserTracker();

  if (tf_plane_) {
    odom_point_origin_.header.frame_id = "/odom";
    odom_point_ez_.header.frame_id = "/odom";

    odom_point_origin_.point.x = 0.0;
    odom_point_origin_.point.y = 0.0;
    odom_point_origin_.point.z = 0.0;

    odom_point_ez_.point.x = 0.0;
    odom_point_ez_.point.y = 0.0;
    odom_point_ez_.point.z = 1.0;

    ROS_INFO("TF check");

    try {
      tfListener_.waitForTransform(odom_point_origin_.header.frame_id, frame_id, ros::Time::now(), ros::Duration(1.0));
      tfListener_.transformPoint(frame_id, odom_point_origin_, optical_point_origin_);
      optical_point_origin_.header.frame_id = frame_id;

      tfListener_.waitForTransform(odom_point_ez_.header.frame_id, frame_id, ros::Time::now(), ros::Duration(1.0));
      tfListener_.transformPoint(frame_id, odom_point_ez_, optical_point_ez_);
      optical_point_ez_.header.frame_id = frame_id;
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("TF check unsuccessful %s: %s", ros::this_node::getName().c_str(), ex.what());
      return;
    }
    ROS_INFO("TF check successful");
  }
}
/////////////////////////////////////////////////////////////////////
SquirrelTracker::~SquirrelTracker() {
  printf("Destructor call!!!");
  shutdownNite();
  shutdownOpenNI();
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::initOpenNI() {
//  ROS_INFO("Initialize OpenNI....\r\n");
  ostatus = openni::OpenNI::initialize();
  if (ostatus != openni::STATUS_OK) {
    ROS_FATAL("ERROR: #%d, %s", ostatus, openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
//-----------------------------------------------------------------
//  ROS_INFO("Opening first device ...\r\n");
  ostatus = device.open(openni::ANY_DEVICE);
  if (ostatus != openni::STATUS_OK) {
    ROS_FATAL("ERROR: #%d, %s", ostatus, openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
//  ROS_INFO("%s Opened, Completed.\r\n", device.getDeviceInfo().getName());
//------------------------------------------------------------------
//  ROS_INFO("Create DepthSensor ...\r\n");
  ostatus = depthSensor.create(device, openni::SENSOR_DEPTH);
  if (ostatus != openni::STATUS_OK) {
    ROS_FATAL("ERROR: #%d, %s", ostatus, openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
//------------------------------------------------------------------
//  ROS_INFO("Mirroring Enabled = false ...\r\n");
  ostatus = depthSensor.setMirroringEnabled(false);
  if (ostatus != openni::STATUS_OK) {
    ROS_FATAL("ERROR: #%d, %s", ostatus, openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
//------------------------------------------------------------------
//  ROS_INFO("DepthSensor start ...\r\n");
  ostatus = depthSensor.start();
  if (ostatus != openni::STATUS_OK) {
    ROS_FATAL("ERROR: #%d, %s", ostatus, openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::initNITE() {
//  ROS_INFO("Initialize NiTE....\r\n");
  nstatus = nite::NiTE::initialize();
  if (nstatus != nite::STATUS_OK) {
    ROS_FATAL("Could not initialize Nite. \n %s\r\n", openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
  nstatus = uTracker.create();
  if (nstatus != nite::STATUS_OK) {
    ROS_FATAL("Could not create UserTracker. \n %s\r\n", openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
  nstatus = hTracker.create();
  if (nstatus != nite::STATUS_OK) {
    ROS_FATAL("Could not create HandTracker. \n %s\r\n", openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::initUserTracker() {
  initOpenNI();
  initNITE();
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::shutdownOpenNI() {
//  ROS_INFO("Close Device and Shutdown OpenNI ...\r\n");
  device.close();
  openni::OpenNI::shutdown();
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::shutdownNite() {
//  ROS_INFO("Destroy UserTracker and Shutdown Nite ...\r\n");
  uTracker.destroy();
  nite::NiTE::shutdown();
}

////////////////////////////////////////////////////////////////////
void SquirrelTracker::setFrameIDParameter() {
  pcl_msg->header.frame_id = frame_id;
  detectedPoseArray.header.frame_id = frame_id;
  pointingPoint.header.frame_id = frame_id;
  squirrelHeadHand.header.frame_id = frame_id;
}
////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishTransformOfPoint(const nite::UserId& userID, const nite::Point3f& point,
                                              const std::string& frame_id, const std::string& child_frame_id,
                                              const ros::Time& timestamp) {
  double x = point.x / 1000;
  double y = -point.y / 1000;
  double z = point.z / 1000;

  transform.setOrigin(tf::Vector3(x, y, z));

  char child_frame_no[128];
  std::snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), (int)(userID));
  tfBraodcaster.sendTransform(tf::StampedTransform(transform, timestamp, frame_id, child_frame_no));
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishTransformOfJoint(const nite::UserId& userID, const nite::SkeletonJoint& joint,
                                              const std::string& frame_id, const std::string& child_frame_id,
                                              const ros::Time& timestamp) {
  double x = joint.getPosition().x / 1000;
  double y = -joint.getPosition().y / 1000;
  double z = joint.getPosition().z / 1000;

  transform.setOrigin(tf::Vector3(x, y, z));
  char child_frame_no[128];
  std::snprintf(child_frame_no, sizeof(child_frame_no), "%s", child_frame_id.c_str());
  tfBraodcaster.sendTransform(tf::StampedTransform(transform, timestamp, frame_id, child_frame_no));
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishPoseStampedArrayOfUsers(const nite::Point3f& point, const ros::Time& timestamp) {
  detectedPose.orientation.x = 0.5;
  detectedPose.orientation.y = -0.5;
  detectedPose.orientation.z = -0.5;
  detectedPose.orientation.w = 0.5;

  detectedPose.position.x = point.x / 1000;
  detectedPose.position.y = -point.y / 1000;
  detectedPose.position.z = point.z / 1000;
  detectedPoseArray.header.stamp = timestamp;
  detectedPoseArray.poses.push_back(detectedPose);
  pubPSA.publish(detectedPoseArray);
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishPointingPoint(const nite::Point3f& point, const ros::Time& timestamp) {
  pointingPoint.point.x = point.x / 1000;
  pointingPoint.point.y = -point.y / 1000;
  pointingPoint.point.z = point.z / 1000;
  pointingPoint.header.stamp = timestamp;

  pubPP.publish(pointingPoint);
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishHeadHandMsgs(const nite::Point3f& head, const nite::Point3f& hand,
                                          const ros::Time& timestamp) {
  squirrelHeadHand.head.x = head.x / 1000;
  squirrelHeadHand.head.y = -head.y / 1000;
  squirrelHeadHand.head.z = head.z / 1000;

  squirrelHeadHand.hand.x = hand.x / 1000;
  squirrelHeadHand.hand.y = -hand.y / 1000;
  squirrelHeadHand.hand.z = hand.z / 1000;

  squirrelHeadHand.header.stamp = timestamp;

  pubPHH.publish(squirrelHeadHand);
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::convertDepthToFilteredPointcloud() {

  openni::VideoFrameRef userDepthFrame = userFrame.getDepthFrame();
  openni::DepthPixel zeroPixel = 0;
  int countZeroPixel = 0;
  openni::DepthPixel* depthCell;
  unsigned int nmbrRow = userDepthFrame.getWidth();
  float dX, dY, dZ;
  nite::UserMap usersMap = userFrame.getUserMap();
  for (unsigned int y = 0; y < userDepthFrame.getHeight(); y++) {
    depthCell = (openni::DepthPixel*)((char*)userDepthFrame.getData() + (y * userDepthFrame.getStrideInBytes()));
    nite::UserId* userPixel = (nite::UserId*)((char*)usersMap.getPixels() + (y * usersMap.getStride()));
    for (unsigned int x = 0; x < userDepthFrame.getWidth(); ++x, ++depthCell, ++userPixel) {
      if (*userPixel != 0 || *depthCell == 0) {
        ++countZeroPixel;
      } else if (*userPixel == 0 && *depthCell != 0) {
        ostatus = openni::CoordinateConverter::convertDepthToWorld(depthSensor, (float)x, (float)y, (float)(*depthCell),
                                                                   &dX, &dY, &dZ);
        if (ostatus != openni::STATUS_OK) {
          ROS_ERROR("ERROR: #%d, %s", ostatus, openni::OpenNI::getExtendedError());
          return;
        }
        pcl_msg->points.push_back(pcl::PointXYZ(dX / 1000, -dY / 1000, dZ / 1000));
      }
    }
  }
  pcl_msg->width = (userDepthFrame.getWidth() * userDepthFrame.getHeight()) - countZeroPixel;
  return;
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishSkeleton(const nite::UserId& userID, const nite::Skeleton& userSkeleton,
                                      const std::string& frame_id, const ros::Time& timestamp) {
  if (nstatus == nite::STATUS_OK && userSkeleton.getState() == nite::SKELETON_TRACKED) {
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_HEAD), frame_id, "head", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_NECK), frame_id, "neck", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_TORSO), frame_id, "torso", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_SHOULDER), frame_id, "left_shoulder",
                            timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_ELBOW), frame_id, "left_elbow", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_HAND), frame_id, "left_hand", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_SHOULDER), frame_id, "right_shoulder",
                            timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_ELBOW), frame_id, "right_elbow", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_HAND), frame_id, "right_hand", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_HIP), frame_id, "left_hip", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_KNEE), frame_id, "left_knee", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_FOOT), frame_id, "left_foot", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_HIP), frame_id, "right_hip", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_KNEE), frame_id, "right_knee", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_FOOT), frame_id, "right_foot", timestamp);
  }
}

/////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishPtCld2(const ros::Time& timestamp) {
  convertDepthToFilteredPointcloud();
#if ROS_VERSION_MINIMUM(1, 11, 1)  // indigo
  pcl_conversions::toPCL(timestamp, pcl_msg->header.stamp);
#else //hydro
  pcl_msg->header.stamp = timestamp.toNSec();
#endif
//   only indigo line below
//  pcl_conversions::toPCL(timestamp, pcl_msg->header.stamp);
  pubPC.publish(pcl_msg);
  pcl_msg->points.clear();

}
/////////////////////////////////////////////////////////////////////
bool SquirrelTracker::detectWavingUser(const nite::UserId& userID) {
  nite::UserMap usersMap = userFrame.getUserMap();
  nstatus = hTracker.readFrame(&handFrame);
  if (nstatus != nite::STATUS_OK || !handFrame.isValid()) {
    return false;
  }
  const nite::Array<nite::GestureData>& gestures = handFrame.getGestures();
  for (int i = 0; i < gestures.getSize(); ++i) {
    if (gestures[i].getType() == nite::GESTURE_WAVE && gestures[i].isComplete()) {
      nite::HandId handId;
      nstatus = hTracker.startHandTracking(gestures[i].getCurrentPosition(), &handId);
    }
  }
  const nite::Array<nite::HandData>& hands = handFrame.getHands();
  for (int i = 0; i < hands.getSize(); ++i) {
    if (hands[i].isTracking()) {
      float posX, posY;
      nstatus = hTracker.convertHandCoordinatesToDepth(hands[i].getPosition().x, hands[i].getPosition().y,
                                                       hands[i].getPosition().z, &posX, &posY);
      if (nstatus == nite::STATUS_OK) {
        nite::UserId* userIdPtr = (nite::UserId*)((char*)usersMap.getPixels() + ((int)posY * usersMap.getStride()))
            + (int)posX;
        if (userID == *userIdPtr) {
          wavingUserID = userID;
          handFrame.release();
          return true;
        }
      }
    }
  }
  return false;
}

/////////////////////////////////////////////////////////////////////
void SquirrelTracker::onNewFrame(nite::UserTracker& uTracker) {
  nstatus = uTracker.readFrame(&userFrame);
  ros::Time timestamp = ros::Time::now();

  nite::Plane floor;

  if (!tf_plane_) {
    floor = userFrame.getFloor();
  } else {
    try {
      tfListener_.waitForTransform(odom_point_origin_.header.frame_id, frame_id, timestamp, ros::Duration(1.0));
      tfListener_.transformPoint(frame_id, odom_point_origin_, optical_point_origin_);

      tfListener_.waitForTransform(odom_point_ez_.header.frame_id, frame_id, timestamp, ros::Duration(1.0));
      tfListener_.transformPoint(frame_id, odom_point_ez_, optical_point_ez_);
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
      return;
    }

    floor.point.x = optical_point_origin_.point.x * 1000;
    floor.point.y = -optical_point_origin_.point.y * 1000;
    floor.point.z = optical_point_origin_.point.z * 1000;

    floor.normal.x = (optical_point_ez_.point.x - optical_point_origin_.point.x) * 1000;
    floor.normal.y = (-optical_point_ez_.point.y + optical_point_origin_.point.y) * 1000;
    floor.normal.z = (optical_point_ez_.point.z - optical_point_origin_.point.z) * 1000;

  }

  if (nstatus != nite::STATUS_OK || !userFrame.isValid()) {
    return;
  }
  if (pub_filtered_pc_) {
    publishPtCld2(timestamp);
  }

  const nite::Array<nite::UserData>& users = userFrame.getUsers();
  if (track_wave_user_) {
    if (!ISWAVEDETECTED) {
      int j = 0;
      for (int i = 0; i < users.getSize(); ++i) {
        if (!users[i].isVisible()) {
          ++j;
        }
      }
      if (j > 0)
        publishedState.state = squirrel_person_tracker_msgs::State::NO_USER;
    }
    for (int i = 0; i < users.getSize(); ++i) {
      nite::UserId userID = users[i].getId();
      if (users[i].isVisible()) {
        if (publishedState.state == squirrel_person_tracker_msgs::State::NO_USER) {
          publishedState.state = squirrel_person_tracker_msgs::State::VISIBLE_USER;
        }
        publishPoseStampedArrayOfUsers(users[i].getCenterOfMass(), timestamp);
//      publishTransformOfPoint(userID, users[i].getCenterOfMass(), frame_id, "detected_pose", timestamp);
      }

      if (ISWAVEDETECTED && SWITCHSKELTRACKON) {
        uTracker.startSkeletonTracking(wavingUserID);
        SWITCHSKELTRACKON = false;
      }
      if (!ISWAVEDETECTED) {
        if (detectWavingUser(userID)) {

          publishedState.state = squirrel_person_tracker_msgs::State::WAVE_USER;
          ISWAVEDETECTED = true;
          ISWAVEUSERVISBLE = true;
          hTracker.stopGestureDetection(nite::GESTURE_WAVE);
        }
      }
      if (ISWAVEDETECTED && wavingUserID == userID) {
        nite::Skeleton userSkeleton = users[i].getSkeleton();
        if (userSkeleton.getState() == nite::SKELETON_TRACKED) {
          if (publishedState.state != squirrel_person_tracker_msgs::State::POINT_USER) {
            publishedState.state = squirrel_person_tracker_msgs::State::SKEL_TRACK_USER;
          }
          if (publish_wuser_skeleton_) {
            publishSkeleton(wavingUserID, userSkeleton, frame_id, timestamp);
          }

          nite::Point3f pointingHead;
          nite::Point3f pointingHand;
          if (pDetector.isFloorPoint(userSkeleton, floor, floorPoint, pointingHead, pointingHand)) {
            publishedState.state = squirrel_person_tracker_msgs::State::POINT_USER;
            publishPointingPoint(floorPoint, timestamp);
            publishHeadHandMsgs(pointingHead, pointingHand, timestamp);
//          publishTransformOfPoint(wavingUserID, point, frame_id, "PointingPoint", timestamp);
          } else {
            publishedState.state = squirrel_person_tracker_msgs::State::SKEL_TRACK_USER;
          }
        } else {
          ROS_INFO("SKELETON STATE: %d", userSkeleton.getState());
//        uTracker.stopSkeletonTracking(wavingUserID);
//        uTracker.startSkeletonTracking(wavingUserID);
          SWITCHSKELTRACKON = true;
        }
      }
      if (!users[i].isVisible() && wavingUserID == userID && ISWAVEUSERVISBLE) {
        ISWAVEUSERVISBLE = false;
        ROS_INFO("Wave user is not visible");
        beginUnvis = timestamp.toSec();
      }
      if (users[i].isVisible() && wavingUserID == userID && ISWAVEDETECTED) {
        ISWAVEUSERVISBLE = true;
      }
      if (!ISWAVEUSERVISBLE && wavingUserID == userID && (timestamp.toSec() - beginUnvis) > durationUnvis) {
        ROS_INFO("Duration is over");
        publishedState.state = squirrel_person_tracker_msgs::State::NO_USER;
        ISWAVEDETECTED = false;
        hTracker.startGestureDetection(nite::GESTURE_WAVE);
        SWITCHSKELTRACKON = true;
        uTracker.stopSkeletonTracking(wavingUserID);
        wavingUserID = 0;
      }
    }
    /////PoseArrayTest
    pubState.publish(publishedState);
    detectedPoseArray.poses.clear();

  }
  publishUsersData(users, timestamp, floor);

  userFrame.release();
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::runSquirrelTracker() {
  setFrameIDParameter();
  hTracker.startGestureDetection(nite::GESTURE_WAVE);
  uTracker.addNewFrameListener(this);
}
void SquirrelTracker::stopSquirrelTracker() {
  ISWAVEDETECTED = false;
  SWITCHSKELTRACKON = true;
  ISWAVEUSERVISBLE = false;
  wavingUserID = 0;
  publishedState.state = squirrel_person_tracker_msgs::State::NO_USER;
  hTracker.stopGestureDetection(nite::GESTURE_WAVE);
  uTracker.removeNewFrameListener(this);
}

void SquirrelTracker::publishUsersData(const nite::Array<nite::UserData>& users, const ros::Time& timestamp,   nite::Plane& floor
) {
  squirrel_person_tracker_msgs::SkeletonVector skel_vec;
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = timestamp;
  skel_vec.header = header;
  for (int i = 0; i < users.getSize(); ++i) {
    nite::UserId userID = users[i].getId();
    if (users[i].isLost()) {
      uTracker.stopSkeletonTracking(userID);
    } else if (users[i].isVisible()) {
      if (users[i].isNew()) {
        uTracker.startSkeletonTracking(userID);
      }
      squirrel_person_tracker_msgs::Skeleton skeleton;
      nite::Skeleton user_skel = users[i].getSkeleton();
      if (user_skel.getState() == nite::SKELETON_TRACKED) {
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_HEAD), frame_id, "head", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_NECK), frame_id, "neck", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_TORSO), frame_id, "torso", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_LEFT_SHOULDER), frame_id, "left_shoulder",
                             timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_LEFT_ELBOW), frame_id, "left_elbow", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_LEFT_HAND), frame_id, "left_hand", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_RIGHT_SHOULDER), frame_id, "right_shoulder",
                             timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_RIGHT_ELBOW), frame_id, "right_elbow", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_RIGHT_HAND), frame_id, "right_hand", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_LEFT_HIP), frame_id, "left_hip", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_LEFT_KNEE), frame_id, "left_knee", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_LEFT_FOOT), frame_id, "left_foot", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_RIGHT_HIP), frame_id, "right_hip", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_RIGHT_KNEE), frame_id, "right_knee", timestamp));
        skeleton.skeleton_joints.push_back(
            transformOfJoint(userID, user_skel.getJoint(nite::JOINT_RIGHT_FOOT), frame_id, "right_foot", timestamp));
        skeleton.user_id = userID;


        nite::Point3f pointingHead;
        nite::Point3f pointingHand;
        if (pDetector.isFloorPoint(user_skel, floor, floorPoint, pointingHead, pointingHand)) {
          skeleton.isPointing = true;
          skeleton.pointingPoint.header = header;
          skeleton.pointingPoint.point.x = floorPoint.x / 1000;
          skeleton.pointingPoint.point.y = -floorPoint.y / 1000;
          skeleton.pointingPoint.point.z = floorPoint.z / 1000;

        } else {
          skeleton.isPointing = false;
          skeleton.pointingPoint.header = header;
          skeleton.pointingPoint.point.x = 0;
          skeleton.pointingPoint.point.y = 0;
          skeleton.pointingPoint.point.z = 0;
        }
      }
      skel_vec.skeleton_vector.push_back(skeleton);
    }
  }
  pubSkelVec.publish(skel_vec);
}
squirrel_person_tracker_msgs::SkeletonJoint SquirrelTracker::transformOfJoint(const nite::UserId& userID,
                                                                              const nite::SkeletonJoint& joint,
                                                                              const std::string& frame_id,
                                                                              const std::string& child_frame_id,
                                                                              const ros::Time& timestamp) {
  double x = joint.getPosition().x / 1000;
  double y = -joint.getPosition().y / 1000;
  double z = joint.getPosition().z / 1000;
  squirrel_person_tracker_msgs::SkeletonJoint joint_msg;
  geometry_msgs::TransformStamped gtransform;
  gtransform.transform.translation.x = x;
  gtransform.transform.translation.y = y;
  gtransform.transform.translation.z = z;

//  char child_frame_no[128];
//  std::snprintf(child_frame_no, sizeof(child_frame_no), "%s", child_frame_id.c_str());
  gtransform.child_frame_id = child_frame_id;
  gtransform.header.stamp = timestamp;
  gtransform.header.frame_id = frame_id;
  joint_msg.joint = gtransform;
  joint_msg.position_confidence = joint.getPositionConfidence();

  return joint_msg;
}
