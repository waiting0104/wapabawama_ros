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
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <cmath>
#include <lgv.h>
#include <perspective.h>

#define CV_SHOW

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
  std::string window_name;
  // Define Sync Policy
  image_transport::Subscriber image_sub_;
  ros::Subscriber bbox_sub_;

  cv_bridge::CvImagePtr cv_ptr;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  ros::Publisher lgv_pub_;
  image_transport::Publisher image_pub_;
  std::vector<darknet_ros_msgs::BoundingBox> boxes;
  /* int biasx, biasy; */
  float _alpha, _beta, orig_cx, orig_cy, d, biasx, biasy;
  int box_y_limit;
  int box_expand;

public:
  LgvDetector() : it_(nh_),
                  tfListener(tfBuffer)
  {
    ros::NodeHandle pn_("~");
    pn_.param<std::string>("image", sub_image_topic, "/camera/image_raw");
    pn_.param<std::string>("bbox", sub_bbox_topic, "/darknet_ros/bounding_boxes");
    pn_.param<std::string>("window", window_name, "window");
    pn_.param<float>("alpha", _alpha, 0);
    pn_.param<float>("beta", _beta, 0);
    pn_.param<float>("cx", orig_cx, 0);
    pn_.param<float>("cy", orig_cy, 0);
    pn_.param<float>("d", d, 0);
    pn_.param<float>("biasx", biasx, 0);
    pn_.param<float>("biasy", biasy, 0);
    pn_.param<int>("box_y_limit", box_y_limit, 0);
    pn_.param<int>("box_expand", box_expand, 0);
    bbox_sub_ = nh_.subscribe(sub_bbox_topic, 10, &LgvDetector::boxcallback, this);
    image_sub_ = it_.subscribe(sub_image_topic, 1, &LgvDetector::imagecallback, this);

    image_pub_ = it_.advertise("pose_img", 1);

#ifdef CV_SHOW
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, 1980 / 2, 1080 / 2);
#endif // CV_SHOW
  }

  ~LgvDetector()
  {
#ifdef CV_SHOW
    cv::destroyWindow(OPENCV_WINDOW);
#endif // CV_SHOW
  }
  void imagecallback(const sensor_msgs::ImageConstPtr &image_msg)
  {

    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    if (!boxes.empty())
    {

      for (auto &box : boxes)
      {
        // Get Bboxs and pub lgvs
        /* if ( box.ymin < box_y_limit || box.ymax > 1920- box_y_limit) continue; */
        /* int bbox_xmin = box.xmin - box_expand; */
        /* int bbox_ymin = box.ymin - box_expand; */
        /* int bbox_xmax = box.xmax + box_expand; */
        /* int bbox_ymax = box.ymax + box_expand; */
        /* bbox_xmin = bbox_xmin < 0 ? 0 : bbox_xmin; */
        /* bbox_ymin = bbox_ymin < 0 ? 0 : bbox_ymin; */
        /* bbox_xmax = bbox_xmax > 1920 ? 1920 : bbox_xmax; */
        /* bbox_ymax = bbox_ymax > 1080 ? 1080 : bbox_ymax; */
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
        cv::Point2d tf_vec = tf_persp_vec_v2(cv::Point2d(vx, vy));
        cv::Point2d tf_cen = tf_persp_v2(cv::Point2d(cx, cy), _alpha / _beta, orig_cx, orig_cy, d);
        tf_cen *= 0.001; // To mm scale
        float show_cx = tf_cen.x + biasx;
        float show_cy = tf_cen.y + biasy;

        // std::cout << "cx: " << show_cx << "cy: " << show_cy << std::endl;
        int vec_size = 100;
        cv::rectangle(cv_ptr->image, new_roi, cv::Scalar(0, 0, 255), 3);
        auto p1 = cv::Point(cx + vx * vec_size, cy + vy * vec_size);
        auto p2 = cv::Point(cx - vx * vec_size, cy - vy * vec_size);
        // cv::line(cv_ptr->image, p1, p2, cv::Scalar(0, 0, 200), 3, 4);
      }
    }

    cv::imshow(window_name, cv_ptr->image);
    cv::waitKey(3);
    std::vector<darknet_ros_msgs::BoundingBox>().swap(boxes);
  }
  void boxcallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
  {

    // try {
    //   cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    // } catch (cv_bridge::Exception& e) {
    //   ROS_ERROR("cv_bridge exception: %s", e.what());
    //   return;
    // }

    boxes = bbox_msg->bounding_boxes;

    /* lgvs.poses.clear(); */

#ifdef CV_SHOW

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