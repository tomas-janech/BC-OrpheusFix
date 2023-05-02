#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <initializer_list>
#include <vector>


#define SegImagesPath "/home/tomas_janech/bc/ros/SegImgs/"

class repeat: public rclcpp::Node{
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr recImg;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr calOut;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr newImage;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr segImage;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped t;

    int imCount = 1;

    sensor_msgs::msg::CameraInfo cal;

/**
 * \brief Fills out camerainfo message
 * \note Not original code
 * \author Matteo Munaro
*/
    void fillCameraInfoMessage(std::string base_name, sensor_msgs::msg::CameraInfo& camera_info_msg ){
        // Read calibration data and fill camera_info message:
        camera_info_msg.width = this->get_parameter(base_name + "/image_width").get_value<int>();
        camera_info_msg.height = this->get_parameter(base_name + "/image_height").get_value<int>();

        camera_info_msg.distortion_model = this->get_parameter(base_name + "/distortion_model").get_value<std::string>();

        std::vector<double> camera_matrix = this->get_parameter(base_name + "/camera_matrix/data").get_value<std::vector<double>>();
        for (unsigned int i = 0; i < camera_matrix.size(); i++)
            camera_info_msg.k[i] = camera_matrix[i];

        std::vector<double> rectification_matrix = this->get_parameter(base_name + "/rectification_matrix/data").get_value<std::vector<double>>();
        for (unsigned int i = 0; i < rectification_matrix.size(); i++)
            camera_info_msg.r[i] = rectification_matrix[i];

        std::vector<double> projection_matrix = this->get_parameter(base_name + "/projection_matrix/data").get_value<std::vector<double>>();
        for (unsigned int i = 0; i < projection_matrix.size(); i++)
            camera_info_msg.p[i] = projection_matrix[i];

        std::vector<double> distortion_coefficients = this->get_parameter(base_name + "/distortion_coefficients/data").get_value<std::vector<double>>();
        for (unsigned int i = 0; i < distortion_coefficients.size(); i++)
            camera_info_msg.d.push_back(distortion_coefficients[i]);
    } 

/**
 * \brief Fomrats file name
 * \details Filename format: xxxxx.bmp (x - file number(always 5 chars))
 * \author Janech Tomas
*/
    std::string formatName(std::string defaultName, int i, std::string extension){
        if(i < 10)
            return defaultName + "0000" + std::to_string(i) + extension;
        else if(i < 100)
            return defaultName + "000" + std::to_string(i) + extension;
        else if(i < 1000)
            return defaultName + "00" + std::to_string(i) + extension;
        else if(i < 10000)
            return defaultName + "0" + std::to_string(i) + extension;
        else
            return defaultName + std::to_string(i) + extension;
    }

/**
 * \details Publishes ROS2 image message from OpenCV matrix
 * \author Janech 
*/
    void publishCVimage(cv::Mat mat, std::string encoding, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher){
        cv_bridge::CvImage cv_img;
        cv_img.image = mat;
        cv_img.encoding = encoding;

        sensor_msgs::msg::Image::SharedPtr ImagePublish = cv_img.toImageMsg();

        publisher->publish(*ImagePublish);
    }

    void resend(const sensor_msgs::msg::Image &msg){
    
        t.header.stamp = msg.header.stamp;
        tf_broadcaster_->sendTransform(t);
        
        cal.header = msg.header;
        cal.header.frame_id = "orpheus_1_realsense_3";
        calOut->publish(cal);
        
        sensor_msgs::msg::Image newImg;
        newImg = msg;
        newImg.header.frame_id = "orpheus_1_realsense_3";
        newImage->publish(newImg);

        cv::Mat mat = cv::imread(formatName(SegImagesPath,imCount,".bmp"), cv::IMREAD_GRAYSCALE);
        if (mat.empty())
            RCLCPP_WARN(this->get_logger(), "Failed to load segmentation image %s", formatName(SegImagesPath,imCount,".bmp"));
        
        publishCVimage(mat, sensor_msgs::image_encodings::MONO8, segImage);

        imCount++;
    }

public:
    repeat():Node("OrpheusFix"){
        this->declare_parameter<int>("camera_calibration/image_width",640);
        this->declare_parameter<int>("camera_calibration/image_height",480);
        this->declare_parameter<std::string>("camera_calibration/distortion_model","plumb_bob");
        this->declare_parameter<std::vector<double>>("camera_calibration/camera_matrix/data",{605.3881, 0., 328.79021, 0., 603.00588, 248.93252, 0., 0., 1.});
        this->declare_parameter<std::vector<double>>("camera_calibration/rectification_matrix/data",{1., 0., 0., 0., 1., 0., 0., 0., 1.});
        this->declare_parameter<std::vector<double>>("camera_calibration/projection_matrix/data",{617.11511, 0., 330.46094, 0., 0., 614.2323, 250.81306, 0., 0. , 0., 1., 0.});
        this->declare_parameter<std::vector<double>>("camera_calibration/distortion_coefficients/data",{0.104739, -0.155903, 0.005916, 0.003733, 0.000000});

        recImg = this->create_subscription<sensor_msgs::msg::Image>(
            "/orpheus_1/sensors/realsense_1/image_raw",
            10, 
            std::bind(&repeat::resend, this, std::placeholders::_1));

        calOut = this->create_publisher<sensor_msgs::msg::CameraInfo>("/orpheus_1/sensors/realsense_2/camera_info", 10);
        newImage = this->create_publisher<sensor_msgs::msg::Image>("/orpheus_1/sensors/realsense_2/image_raw", 10);
        segImage = this->create_publisher<sensor_msgs::msg::Image>("/algorithms/segmentation", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        fillCameraInfoMessage("camera_calibration", cal);

        t.header.frame_id = "orpheus_1_realsense_1";
        t.child_frame_id = "orpheus_1_realsense_3";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = -0.5;
        t.transform.rotation.y = 0.5;
        t.transform.rotation.z = -0.5;
        t.transform.rotation.w = 0.5;
    };


};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<repeat>());
    rclcpp::shutdown();
    return 0;
}
