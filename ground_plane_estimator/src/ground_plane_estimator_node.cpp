#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ground_plane_estimator/ground_plane_estimator_node.hpp"

using namespace std::chrono_literals;

std::shared_ptr<GroundPlaneEstimatorNode> node_;
//std::shared_ptr<image_transport::ImageTransport> it_;

GroundPlaneEstimatorNode::GroundPlaneEstimatorNode()
: Node("ground_plane_estimator_node"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&GroundPlaneEstimatorNode::topic_callback, this, std::placeholders::_1));
    // timer_ = this->create_wall_timer(500ms, std::bind(&GroundPlaneEstimatorNode::timer_callback, this));
    // rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    // Bind to static method
    //transport_image_subscription_ = image_transport::create_subscription(node_.get(), "/d435/camera/aligned_depth_to_color", &GroundPlaneEstimatorNode::image_callback_static, "raw", custom_qos);
    //transport_image_subscription_ = it_->subscribe("/d435/camera/aligned_depth_to_color", 1, &GroundPlaneEstimatorNode::image_callback);
    //sub_image = this->create_subscription<sensor_msgs::msg::Image>("/d435/camera/aligned_depth_to_color/image_raw", 10, std::bind(&GroundPlaneEstimatorNode::image_callback, this, std::placeholders::_1));
    //sub_image = this->create_subscription<sensor_msgs::msg::Image>("/d435/camera/color/image_raw", 10, std::bind(&GroundPlaneEstimatorNode::image_callback, this, std::placeholders::_1));
    // sub_image = message_filters::Subscriber<sensor_msgs::msg::Image> (nh, "/camera/color/image_raw", 1);

    
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_image, sub_points);
   
    sub_image_ = std::make_unique<sub_image>(this, "/d435/camera/color/image_raw");
    sub_points_ = std::make_unique<sub_points>(this, "/d435/camera/pointcloud");
    // sync_sub_ = std::make_unique<sync>(*sub_image_, *sub_points_, 10);
    sync_sub_ = std::make_unique<sync>(MySyncPolicy(10), *sub_image_, *sub_points_);
    sync_sub_->registerCallback(std::bind(&GroundPlaneEstimatorNode::imageColorEdgeDetection, this, std::placeholders::_1, std::placeholders::_2));
  

    // typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_image, sub_points);
    // TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(image_sub, sub_point, 10);
    // sync.registerCallback(boost::bind(&imageColorEdgeDetection, _1, _2));

    // scan_filter_sub_ =
    // std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
    // shared_from_this().get(), scan_topic_, rmw_qos_profile_sensor_data);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/sidewalk_detector/color/image_raw", 1 );
    pub_depth_in = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sidewalk_detector/depth/points_in", 1);
    pub_depth_out = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sidewalk_detector/depth/points_out", 1);    
}

void GroundPlaneEstimatorNode::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

void GroundPlaneEstimatorNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

// Static function, since image transport doesn't support member methods.
// Using global shared pointer for the "node_" instead of "this", since this is a static function.
void GroundPlaneEstimatorNode::image_callback_static(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    printf("image_callback_static...\n");
    RCLCPP_INFO(node_->get_logger(), "Image arrived '%dx%d'", msg->width, msg->height);
    printf("image_callback_static.\n");
}

void GroundPlaneEstimatorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Image arrived '%dx%d'", msg->width, msg->height);
    image_pub_->publish(msg);
}

void GroundPlaneEstimatorNode::imageColorEdgeDetection(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & point_msg) {
    RCLCPP_INFO(this->get_logger(), "Image arrived imageColorEdgeDetection '%dx%d' - Points '%dx%d'", msg->width, msg->height, point_msg->width, point_msg->height);
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    // acute the egde in the image
    cv::Mat imgResult = imageAcuting(cv_ptr);
    // detect the lines in the image
    vector<cv::Vec4i> lines = edgeLineDetection(imgResult);
    // choose the line which is most likely to be the sidewalk edge
    sidewalkEdgePicking(lines);
    // After filter, two line is obtained, represented by
    // left : lp_x1, lp_y1, lp_x2, lp_y2;
    // right: rp_x1, rp_y1, rp_x2, rp_y2;
    cv_ptr = showImageWithMark(cv_ptr);
    // publish processed image raw data
    image_pub_->publish(*cv_ptr->toImageMsg());

    // now deal with the point cloud
    sensor_msgs::msg::PointCloud2 msg_in;
    sensor_msgs::msg::PointCloud2 msg_out;

    // convert form sensor_msgs::PointCloud2 to PCLPointCloud2
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*point_msg, pcl_pc);

    // convert form PCLPointCloud2 to PCLPointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());

    pcl::ExtractIndices<pcl::PointXYZRGB> extract_in;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_out;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices ());

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.1);

    int i = 0;
    int n = (int) cloud->points.size();
    while (cloud->points.size() > 0.5 * n) {
        seg.setInputCloud(cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the inliers
        extract_in.setInputCloud (cloud);
        extract_in.setIndices (inliers);
        extract_in.setNegative (false);
        extract_in.filter (*cloud_in);
        // Create the filtering object
        extract_in.setNegative (true);
        extract_in.filter (*cloud_out);
        cloud.swap (cloud_out);
        i++;
    }

    // cv_bridge::CvImagePtr cv_ptr;
    // try {
    //     cv_ptr = cv_bridge::toCvCopy( msg, "rgb8" );
    // }
    // catch ( cv_bridge::Exception& e ) {
    //     RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what() );
    //     return;
    // }  
    // for (int i = 0; i < inliers->indices.size(); i++) {
    //     int index = (int) i/(0.75*0.75);
    //     uint32_t x = (index) % (int)WIDTH;
    //     uint32_t y = (index) / (int)WIDTH;
    //     cv_ptr->image.at<cv::Vec3b>(x,y)[0]=255;
    //     cv_ptr->image.at<cv::Vec3b>(x,y)[1]=0;
    //     cv_ptr->image.at<cv::Vec3b>(x,y)[2]=0;
    // }
    // cv::flip( cv_ptr->image, cv_ptr->image, -1 );
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(1);

    // convert form PCLPointCloud to sensor_msgs::PointCloud2
    pcl::toROSMsg(*cloud_in, msg_in);
    pcl::toROSMsg(*cloud_out, msg_out);
    // publich topic
    pub_depth_in->publish(msg_in);
    pub_depth_out->publish(msg_out);
}

cv::Mat GroundPlaneEstimatorNode::imageAcuting(cv_bridge::CvImagePtr cv_ptr) {
    cv::Mat kernel = (cv::Mat_<float>(3,3) <<
        1,  1, 1,
        1, -8, 1,
        1,  1, 1);
    // do the laplacian filtering to acute the edge
    cv::Mat imgLaplacian;
    cv::Mat sharp = cv_ptr->image;
    filter2D(sharp, imgLaplacian, CV_32F, kernel);
    cv_ptr->image.convertTo(sharp, CV_32F);
    cv::Mat imgResult = sharp - imgLaplacian;
    // convert back to 8bits gray scale
    imgResult.convertTo(imgResult, CV_8UC3);
    return imgResult;
}

vector<cv::Vec4i> GroundPlaneEstimatorNode::edgeLineDetection(cv::Mat img) {
    cv::Mat gray, edge;
    cv::cvtColor( img, gray, CV_RGB2GRAY );
    // cv::blur( gray, edge, cv::Size(5,5) );
    cv::Canny( gray, edge, 50, 150, 3);
    vector<cv::Vec4i> lines;
    cv::HoughLinesP( edge, lines, 1, CV_PI/180, 80, 20, 10 );
    return lines;
}

void GroundPlaneEstimatorNode::sidewalkEdgePicking(vector<cv::Vec4i> lines) {
    double lmin = WIDTH/2, rmax = WIDTH/2;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        // if (lines[i][2] == lines[i][0]) continue;
        double alpha = atan2((lines[i][3]-lines[i][1]), (lines[i][2]-lines[i][0]));
        double b = lines[i][3] - lines[i][2] * tan(alpha);
        if ( lines[i][0] > 540 && lines[i][2] > 540 ) { // right part
            if ( (alpha < -0.15*CV_PI/2) && (alpha > -0.85*CV_PI/2) ) {
                double x11, y11;
                (void)x11; (void)y11;
                double x21 = (HEIGHT/2 - b) / tan(alpha);
                double y21 = HEIGHT /2 ;
                if (x21 < rmax) {
                    continue;
                }
                rmax = x21;
                rp_x2 = x21;
                rp_y2 = y21;
                if (tan(alpha) * WIDTH + b <= HEIGHT) {
                    rp_x1 = WIDTH;
                    rp_y1 = tan(alpha) * rp_x1 + b;
                } else {
                    rp_x1 = (HEIGHT - b) / tan(alpha);
                    rp_y1 = HEIGHT;
                }
            }
        } else if ( lines[i][0] < 100 && lines[i][2] < 100 ) { // left part
            if ( (alpha > 0.15*CV_PI/2) && (alpha < 0.85*CV_PI/2) ) {
                double x12, y12;
                (void)x12; (void)y12;
                double x22 = (HEIGHT/2 - b) / tan(alpha);
                double y22 = HEIGHT/2;
                if (x22 > lmin) {
                    continue;
                }
                lmin = x22;
                lp_x2 = x22;
                lp_y2 = y22;
                if (b <= HEIGHT) {
                    lp_x1 = 0.0;
                    lp_y1 = b;
                } else {
                    lp_x1 = (HEIGHT - b) / tan(alpha);
                    lp_y1 = HEIGHT;
                }
            }
        }
    }
}

cv_bridge::CvImagePtr GroundPlaneEstimatorNode::showImageWithMark(cv_bridge::CvImagePtr cv_ptr) {
    cv::Point polygon[1][6];
    polygon[0][0] = cv::Point(0.0, lp_y1);
    polygon[0][1] = cv::Point(lp_x2, lp_y2);
    polygon[0][2] = cv::Point(rp_x2, rp_y2);
    polygon[0][3] = cv::Point(WIDTH, rp_y1);
    polygon[0][4] = cv::Point(WIDTH, 0);
    polygon[0][5] = cv::Point(0.0, 0.0);
    const cv::Point* ppt[1] = { polygon[0] };
    int npt[] = { 6 };
    cv::Mat poly(cv_ptr->image.size(), CV_8UC3);
    cv::fillPoly(poly, ppt, npt, 1, cv::Scalar(0,0,255));
    cv::Mat roi = poly(cv::Rect(0, 0, cv_ptr->image.cols, cv_ptr->image.rows));
    double alpha = 0.7;
    cv::addWeighted(cv_ptr->image, alpha, roi, 1.0 - alpha , 0.0, cv_ptr->image); 

    //cv::flip( cv_ptr->image, cv_ptr->image, -1 );
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(1);
    return cv_ptr;
}

// imageSegment is using image segmentation to detect sidewalk
// now still not work well, still need to test
void GroundPlaneEstimatorNode::imageSegment( const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy( msg, "rgb8");
    }
    catch ( cv_bridge::Exception& e ) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what() );
        return;
    }
    cv::flip( cv_ptr->image, cv_ptr->image, -1 );
    // Create a kernel that we will use for accuting/sharpening our image
    cv::Mat kernel = (cv::Mat_<float>(3,3) <<
        1,  1, 1,
        1, -8, 1,
        1,  1, 1);
    // do the laplacian filtering
    cv::Mat imgLaplacian;
    cv::Mat sharp = cv_ptr->image; // copy source image to another temporary one
    filter2D(sharp, imgLaplacian, CV_32F, kernel);
    cv_ptr->image.convertTo(sharp, CV_32F);
    cv::Mat imgResult = sharp - imgLaplacian;
    // convert back to 8bits gray scale
    imgResult.convertTo(imgResult, CV_8UC3);
    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
    // cv::imshow( OPENCV_WINDOW, imgResult );
    cv_ptr->image = imgResult;
    // Create binary image from source image
    cv::Mat bw;
    cv::cvtColor(cv_ptr->image, bw, CV_RGB2HSV);
    std::vector<cv::Mat> channels;
    cv::split(bw, channels);
    bw = channels[0];
    // cv::cvtColor(cv_ptr->image, bw, CV_RGB2GRAY);
    cv::threshold(bw, bw, 20, 120, CV_THRESH_BINARY | CV_THRESH_OTSU);
    // cv::imshow( OPENCV_WINDOW, bw );
    // Perform the distance transform algorithm
    cv::Mat dist;
    cv::distanceTransform(bw, dist, CV_DIST_L2, 3);
    // Normalize the distance image for range = {0.0, 1.0}
    // so we can visualize and threshold it
    cv::normalize(dist, dist, 0, 1., cv::NORM_MINMAX);
    // cv::imshow(OPENCV_WINDOW, dist);
    // Threshold to obtain the peaks
    // This will be the markers for the foreground objects
    cv::threshold(dist, dist, .4, 1., CV_THRESH_BINARY);
    // Dilate a bit the dist image
    cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8UC1);
    cv::dilate(dist, dist, kernel1);
    // cv::imshow(OPENCV_WINDOW, dist);
    // Create the CV_8U version of the distance image
    // It is needed for findContours()
    cv::Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);
    // Find total markers
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    // Create the marker image for the watershed algorithm
    cv::Mat markers = cv::Mat::zeros(dist.size(), CV_32SC1);
    // Draw the foreground markers
    for (size_t i = 0; i < contours.size(); i++)
        cv::drawContours(markers, contours, static_cast<int>(i), cv::Scalar::all(static_cast<int>(i)+1), -1);
    // Draw the background marker
    cv::circle(markers, cv::Point(5,5), 3, CV_RGB(255,255,255), -1);
    // cv::imshow(OPENCV_WINDOW, markers*10000);
    // Perform the watershed algorithm
    cv::watershed(cv_ptr->image, markers);
    cv::Mat mark = cv::Mat::zeros(markers.size(), CV_8UC1);
    markers.convertTo(mark, CV_8UC1);
    cv::bitwise_not(mark, mark);
    // Generate random colors
    std::vector<cv::Vec3b> colors;
    for (size_t i = 0; i < contours.size(); i++)
    {
        int b = cv::theRNG().uniform(0, 255);
        int g = cv::theRNG().uniform(0, 255);
        int r = cv::theRNG().uniform(0, 255);
        colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
    }
    // Create the result image
    cv::Mat dst = cv::Mat::zeros(markers.size(), CV_8UC3);
    // Fill labeled objects with random colors
    for (int i = 0; i < markers.rows; i++)
    {
        for (int j = 0; j < markers.cols; j++)
        {
            int index = markers.at<int>(i,j);
            if (index > 0 && index <= static_cast<int>(contours.size()))
                dst.at<cv::Vec3b>(i,j) = colors[index-1];
            else
                dst.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
        }
    }
    // Visualize the final image
    //imshow(OPENCV_WINDOW, dst);
    // cv::imshow( OPENCV_WINDOW, cv_ptr->image );
    //cv::waitKey(3);
    image_pub_->publish( cv_ptr->toImageMsg() );
}

int main(int argc, char * argv[])
{
  (void) argc;
  (void) argv;

  printf("hello world ground_plane_estimator package\n");
  rclcpp::init(argc, argv);
  node_ = std::make_shared<GroundPlaneEstimatorNode>();
  //it_ = std::make_shared<image_transport::ImageTransport>(node_);
  rclcpp::spin(node_);
  rclcpp::shutdown();
  return 0;
}
