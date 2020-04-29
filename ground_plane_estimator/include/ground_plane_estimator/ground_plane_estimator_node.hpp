// Copyright (c) 2019 Intel Corporation. All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GROUND_PLANE_ESTIMATOR_NODE_HPP_
#define GROUND_PLANE_ESTIMATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/PointIndices.h>
//#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <math.h>
#include <vector>

using namespace std;
using namespace std::chrono_literals;

static const std::string OPENCV_WINDOW = "Image window";
double HEIGHT = 480.0;
double WIDTH  = 640.0;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> MySyncPolicy;

class GroundPlaneEstimatorNode : public rclcpp::Node
{
  public:
    GroundPlaneEstimatorNode();

  private:

    void timer_callback();
    size_t count_;

    double lp_x1, lp_y1, lp_x2, lp_y2;
    double rp_x1, rp_y1, rp_x2, rp_y2;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_depth_in;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_depth_out;
    
    // Normal subscribers
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;

    // Time synchronizer subscribers
    // message_filters::Subscriber<sensor_msgs::PointCloud2> sub_points;
    // message_filters::Subscriber<sensor_msgs::msg::Image> sub_image;
    // using sync = message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>;

    // Exact time synchronizer subscribers
    using sub_image = message_filters::Subscriber<sensor_msgs::msg::Image>;
    using sub_points = message_filters::Subscriber<sensor_msgs::msg::PointCloud2>;
    using sync = message_filters::Synchronizer<MySyncPolicy>;

    std::unique_ptr<sub_image> sub_image_;
    std::unique_ptr<sub_points> sub_points_;
    std::unique_ptr<sync> sync_sub_;

    void callback(const std_msgs::msg::Header::ConstSharedPtr msg1, const std_msgs::msg::Header::ConstSharedPtr msg2) { std::cout << "callback" << std::endl; };

    // message_filters::TimeSynchronizer<std_msgs::msg::Header, std_msgs::msg::Header> sync;
    // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync;


    //image_transport::Subscriber transport_image_subscription_;

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
    static void image_callback_static(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void imageColorEdgeDetection(const sensor_msgs::msg::Image::ConstSharedPtr& msg, 
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & point_msg);
    cv::Mat imageAcuting(cv_bridge::CvImagePtr cv_ptr);
    vector<cv::Vec4i> edgeLineDetection(cv::Mat img);
    void sidewalkEdgePicking(vector<cv::Vec4i> lines);
    cv_bridge::CvImagePtr showImageWithMark(cv_bridge::CvImagePtr cv_ptr);
    void imageSegment( const sensor_msgs::msg::Image::ConstSharedPtr& msg );
  };

#endif  // GROUND_PLANE_ESTIMATOR_NODE_HPP_