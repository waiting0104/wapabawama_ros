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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <cmath>
#include <lgv.h>

class LgvDetector {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  std::string sub_image_topic;
  std::string sub_bbox_topic;

  // Define Sync Policy
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, 
      darknet_ros_msgs::BoundingBoxes
    > SyncPolicy;
  typedef image_transport::SubscriberFilter ImageSub;
  typedef message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> BboxSub;
  typedef message_filters::Synchronizer< SyncPolicy > Syncr;

  std::shared_ptr<ImageSub> image_sub_;
  std::shared_ptr<BboxSub>  bbox_sub_;
  std::shared_ptr<Syncr>    sync;

  ros::Publisher lgv_pub_;
  image_transport::Publisher image_pub_;

  int box_expand;
  float arrow_length;

public:
  LgvDetector() : it_(nh_) {
    ros::NodeHandle pn_("~");
    pn_.param<std::string>( "image", sub_image_topic, "/camera/image_raw" );
    pn_.param<std::string>( "bbox" , sub_bbox_topic , "/darknet_ros/bounding_boxes" );

    pn_.param<int>(   "box_expand"   , box_expand  , 0 );
    pn_.param<float>( "arrow_length" , arrow_length, 5 );

    image_sub_ = std::make_shared<ImageSub>(it_, sub_image_topic, 1);
    bbox_sub_  = std::make_shared<BboxSub>( nh_, sub_bbox_topic , 10);
    sync       = std::make_shared<Syncr>( SyncPolicy( 10 ), *image_sub_, *bbox_sub_);

    sync->registerCallback( boost::bind( &LgvDetector::callback, this, _1, _2 ) );

    /* lgv_pub_ = nh_.advertise<geometry_msgs::PoseArray>("lgvs", 10); */
    image_pub_ = it_.advertise("pose_img", 1);
  }

  ~LgvDetector() {
    image_sub_.reset();
    bbox_sub_.reset();
    sync.reset();
  }

  void callback(
      const sensor_msgs::ImageConstPtr& image_msg,
      const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg ) {

    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Lgvs main
    geometry_msgs::PoseArray lgvs;

    for (auto& box:bbox_msg->bounding_boxes){

      int bbox_xmin = box.xmin - box_expand;
      int bbox_ymin = box.ymin - box_expand;
      int bbox_xmax = box.xmax + box_expand;
      int bbox_ymax = box.ymax + box_expand;
      bbox_xmin = bbox_xmin < 0 ? 0 : bbox_xmin;
      bbox_ymin = bbox_ymin < 0 ? 0 : bbox_ymin;
      bbox_xmax = bbox_xmax > 1920 ? 1920 : bbox_xmax;
      bbox_ymax = bbox_ymax > 1080 ? 1080 : bbox_ymax;

      cv::Rect roi( bbox_xmin, bbox_ymin, (bbox_xmax - bbox_xmin), (bbox_ymax - bbox_ymin) );
      cv::Mat select_box = cv_ptr->image(roi);

      lgv::Lgvector lgv_ = lgv::fixsample_fit(select_box, lgv::cfn_ratio);

      float cx, cy, vx, vy;
      cx = ( lgv_.x + 1.5 * bbox_xmin + 0.5 * bbox_xmax ) / 2;
      cy = ( lgv_.y + 1.5 * bbox_ymin + 0.5 * bbox_ymax ) / 2;
      vx = lgv_.dx;
      vy = lgv_.dy;


      cv::rectangle(cv_ptr->image, roi, cv::Scalar(0,0,255), 3);

      auto p1 = cv::Point(cx + vx * arrow_length, cy + vy * arrow_length);
      auto p2 = cv::Point(cx - vx * arrow_length, cy - vy * arrow_length);

      cv::line(cv_ptr->image, p1, p2, cv::Scalar(0,0,200), 3, 4);

      /* geometry_msgs::Pose lgv_msg; */
      /* tf2::Quaternion quat_tf; */
      /* quat_tf.setRPY(0,0,atan2(vy, vx) ); */
      /* quat_tf.normalize(); */
      /* lgv_msg.orientation = tf2::toMsg(quat_tf);; */
      /* lgv_msg.position.x = cx; */
      /* lgv_msg.position.y = cy; */

      /* lgvs.poses.push_back(lgv_msg); */
    }
    /* lgvs.header.stamp = ros::Time::now(); */
    /* lgvs.header.frame_id = "map"; */
    /* lgv_pub_.publish(lgvs); */
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "lgv_detector_node");
  LgvDetector lgvd;
  ros::spin();
  return 0;
}
