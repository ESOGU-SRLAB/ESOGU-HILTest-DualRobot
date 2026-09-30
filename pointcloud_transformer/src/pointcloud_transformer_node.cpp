#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include <memory>

namespace pointcloud_transformer
{

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("pointcloud_transformer")
    {
        this->declare_parameter<std::string>("input_topic", "points");
        this->declare_parameter<std::string>("output_topic", "sick_points");
        this->declare_parameter<std::string>("target_frame", "ur10e_depth_optical_frame");
        
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("target_frame", target_frame_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
        
        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 
            10, 
            std::bind(&PointCloudTransformer::pointcloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PointCloud Transformer başlatıldı");
        RCLCPP_INFO(this->get_logger(), "Input: %s -> Output: %s", input_topic_.c_str(), output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Debug: Gelen mesajın frame_id'sini logla
        RCLCPP_INFO_ONCE(this->get_logger(), "Gelen pointcloud frame_id: %s", msg->header.frame_id.c_str());
        
        if (msg->header.frame_id == target_frame_) {
            RCLCPP_DEBUG(this->get_logger(), "Zaten hedef frame'de, dönüşüm gerekmiyor");
            publisher_->publish(*msg);
            return;
        }

        try
        {
            // Transform'un mevcut olup olmadığını kontrol et
            if (!tf_buffer_->canTransform(target_frame_, msg->header.frame_id, 
                                        msg->header.stamp, rclcpp::Duration::from_seconds(0.1))) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Transform mevcut değil: %s -> %s", msg->header.frame_id.c_str(), target_frame_.c_str());
                return;
            }

            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.5));

            // Debug: Transform bilgilerini logla
            RCLCPP_INFO_ONCE(this->get_logger(), "Transform bulundu - Translation: [%.3f, %.3f, %.3f]",
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z);

            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, transform_stamped);
            
            // Başarılı transformasyon mesajı
            RCLCPP_INFO_ONCE(this->get_logger(), "İlk pointcloud başarıyla dönüştürüldü");
            
            publisher_->publish(transformed_cloud);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Transform hatası: %s", ex.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
};

} // namespace pointcloud_transformer

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pointcloud_transformer::PointCloudTransformer>());
    rclcpp::shutdown();
    return 0;
}