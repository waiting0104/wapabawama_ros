/*
 * main.cpp
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <wapabawama_ros/lgv.h>
#include <wapabawama_ros/lgvs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <cmath>
#include <lgv.h>
#include <perspective.h>

//  #define CV_SHOW

/*
 * pabawama(Path Base Water Machine)
 *
 * Sub:
 *  - camera/image_raw
 *  - darknet/bounding_boxes
 *
 * Pub:
 *  - pabawama_ros/lgvs(geometry_msgs::PoseArray)
 */

#ifdef CV_SHOW
static const std::string OPENCV_WINDOW = "Image window";
#endif // CV_SHOW

class LgvDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  std::string sub_image_topic;
  std::string sub_bbox_topic;
  std::string pub_lgvs_topic;
  std::string pub_img_topic;
  // Define Sync Policy
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      darknet_ros_msgs::BoundingBoxes>
      SyncPolicy;
  typedef image_transport::SubscriberFilter ImageSub;
  typedef message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> BboxSub;
  typedef message_filters::Synchronizer<SyncPolicy> Syncr;

  std::shared_ptr<ImageSub> image_sub_;
  std::shared_ptr<BboxSub> bbox_sub_;
  std::shared_ptr<Syncr> sync;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  ros::Publisher lgv_pub_;
  ros::Publisher lgv_pub_tracked;
  image_transport::Publisher image_pub_;

  /* int biasx, biasy; */
  float _alpha, _beta, orig_cx, orig_cy, d, biasx, biasy;
  int box_y_limit;
  int box_expand;
  int xx;

public:
  LgvDetector() : it_(nh_),
                  tfListener(tfBuffer)
  {
    ros::NodeHandle pn_("~");
    pn_.param<std::string>("image", sub_image_topic, "/camera/image_raw");
    pn_.param<std::string>("bbox", sub_bbox_topic, "/darknet_ros/bounding_boxes");
    pn_.param<std::string>("pub_lgvs_topic", pub_lgvs_topic, "lgvs_tracked");
    pn_.param<std::string>("pub_img_topic", pub_img_topic, "pose_img");
    pn_.param<float>("alpha", _alpha, 0);
    pn_.param<float>("beta", _beta, 0);
    pn_.param<float>("cx", orig_cx, 0);
    pn_.param<float>("cy", orig_cy, 0);
    pn_.param<float>("d", d, 0);
    pn_.param<float>("biasx", biasx, 0);
    pn_.param<float>("biasy", biasy, 0);
    pn_.param<int>("box_y_limit", box_y_limit, 0);
    pn_.param<int>("box_expand", box_expand, 0);
    pn_.param<int>("xx", xx, 0);
    image_sub_ = std::make_shared<ImageSub>(it_, sub_image_topic, 1);
    bbox_sub_ = std::make_shared<BboxSub>(nh_, sub_bbox_topic, 10);
    sync = std::make_shared<Syncr>(SyncPolicy(10), *image_sub_, *bbox_sub_);

    sync->registerCallback(boost::bind(&LgvDetector::callback, this, _1, _2));

    lgv_pub_ = nh_.advertise<geometry_msgs::PoseArray>("lgvs", 10);
    lgv_pub_tracked = nh_.advertise<wapabawama_ros::lgvs>(pub_lgvs_topic, 10);
    image_pub_ = it_.advertise(pub_img_topic, 1);

#ifdef CV_SHOW
    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
    cv::resizeWindow(OPENCV_WINDOW, 1980 / 2, 1080 / 2);
#endif // CV_SHOW
  }

  ~LgvDetector()
  {
    image_sub_.reset();
    bbox_sub_.reset();
    sync.reset();
#ifdef CV_SHOW
    cv::destroyWindow(OPENCV_WINDOW);
#endif // CV_SHOW
  }

  void callback(const sensor_msgs::ImageConstPtr &image_msg, const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform("map", "gantry", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    float gantry_x = transformStamped.transform.translation.x;
    float gantry_y = transformStamped.transform.translation.y;

    // Lgvs main
    wapabawama_ros::lgvs lgvs_tracked;

    /* lgvs.poses.clear(); */
    for (auto &box : bbox_msg->bounding_boxes)
    {
      // Get Bboxs and pub lgvs

      cv::Rect roi(box.xmin, box.ymin, (box.xmax - box.xmin), (box.ymax - box.ymin));
      cv::Rect new_roi = lgv::expandSquBox(roi, cv_ptr->image, box_expand);
      cv::Mat select_box = cv_ptr->image(new_roi);
      lgv::Lgvector lgv_ = lgv::fixsample_fit(select_box, lgv::cfn_ratio);
      /* float cx = ( lgv_.x + 1.5 * bbox_xmin + 0.5 * bbox_xmax ) / 2; */
      /* float cy = ( lgv_.y + 1.5 * bbox_ymin + 0.5 * bbox_ymax ) / 2; */
      float cx = (lgv_.x + new_roi.width / 2) / 2 + new_roi.x;
      float cy = (lgv_.y + new_roi.height / 2) / 2 + new_roi.y;
      float vx = lgv_.dx;
      float vy = lgv_.dy;
      if (cy >= 200 && cy <= 1000)
      {
        // Perspective to map frame
        cv::Point2d tf_vec = tf_persp_vec_v2(cv::Point2d(vx, vy));
        cv::Point2d tf_cen = tf_persp_v2(cv::Point2d(cx, cy), _alpha / _beta, orig_cx, orig_cy, d);
        // std::cout<<tf_cen.x<<std::endl;
        tf_cen *= 0.001; // To mm scale

        wapabawama_ros::lgv lgv_msg_tracked;

        // geometry_msgs::Pose lgv_msg;
        tf2::Quaternion quat_tf;
        /* quat_tf.setRPY(0,0,atan2(tf_vec.y, tf_vec.x) ); */
        quat_tf.setRPY(0, 0, atan2(tf_vec.y, -tf_vec.x));
        quat_tf.normalize();

        lgv_msg_tracked.pose.orientation = tf2::toMsg(quat_tf);
        lgv_msg_tracked.pose.position.x = tf_cen.x + gantry_x + biasx;
        lgv_msg_tracked.pose.position.y = tf_cen.y + gantry_y + biasy;
        lgv_msg_tracked.ID = box.id;
        lgv_msg_tracked.count = box.count;
        // Push data here
        lgvs_tracked.lgvs.push_back(lgv_msg_tracked);
      }
    }

    lgvs_tracked.header.stamp = ros::Time::now();
    lgvs_tracked.header.frame_id = "map";
    lgv_pub_tracked.publish(lgvs_tracked);
#ifdef CV_SHOW
    int vec_size = 100;
    cv::rectangle(cv_ptr->image, new_roi, cv::Scalar(0, 0, 255), 3);
    auto p1 = cv::Point(cx + vx * vec_size, cy + vy * vec_size);
    auto p2 = cv::Point(cx - vx * vec_size, cy - vy * vec_size);

    cv::line(cv_ptr->image, p1, p2, cv::Scalar(0, 0, 200), 3, 4);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
#endif // CV_SHOW

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lgv_detector_node");
  LgvDetector lgvd;
  ros::spin();
  return 0;
}
