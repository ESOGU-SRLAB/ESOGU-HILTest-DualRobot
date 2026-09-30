#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <memory>
#include <chrono>
#include <vector>
#include <string>
#include <map>

namespace pose_transformer
{

// Tablodaki her satırı temsil edecek basit bir veri yapısı
struct PoseData
{
    std::string name;
    double x, y, z;       // Pozisyon (metre cinsinden)
    double rx, ry, rz;    // Oryantasyon (radyan cinsinden Euler açıları)
};

class PoseTransformerNode : public rclcpp::Node
{
public:
    PoseTransformerNode() : Node("pose_transformer_node")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 1. ADIM: Tablodaki tüm verileri C++ vektörüne yükle
        // Not: Pozisyon değerleri mm'den metreye çevrilmiştir.
        all_poses_ = {
            {"vr_top", 0.88677, -0.36982, 0.98789, 2.608, -2.351, 2.380},
            {"vr_middle", 0.88677, -0.36982, 0.65129, 2.608, -2.351, 2.380},
            {"vr_below", 0.88677, -0.36982, 0.14709, 2.608, -2.351, 2.380},
            {"right_in", 0.93016, -0.44205, 0.87820, 2.652, -2.266, 2.464},
            {"r_lookleft", 0.93017, -0.44205, 0.87818, 0.355, -4.737, -0.248},
            {"r_looktop", 0.93017, -0.44205, 0.87819, 1.096, 1.273, 1.306},
            {"r_lookright", 0.93017, -0.44205, 0.87817, 2.213, 0.166, 2.324},
            {"r_lookfront", 0.83362, -0.32234, 0.87114, 0.0, 0.0, 0.0}, // RX,RY,RZ yok, 0 varsayıldı
            {"right_out", 0.43442, -0.46829, 0.87820, 2.652, -2.266, 2.464},
            {"intermediate_point", 0.42753, -0.00206, 0.83074, 0.0, 0.0, 0.0}, // RX,RY,RZ yok, 0 varsayıldı
            {"left_in", 0.95184, 0.34283, 0.92150, 2.652, -2.346, 2.332},
            {"l_lookleft", 0.95184, 0.34283, 0.92148, 0.084, -4.650, 0.116},
            {"l_looktop", 0.95184, 0.34284, 0.92147, 1.223, 1.268, 1.143},
            {"l_lookright", 0.95184, 0.34287, 0.92144, 2.260, 0.041, 2.129},
            {"vl_top", 1.01823, 0.21030, 1.07784, 0.050, -4.866, 0.412},
            {"vl_middle", 1.01823, 0.21030, 0.79020, 0.050, -4.866, 0.412},
            {"vl_below", 1.01823, 0.21030, 0.09020, 0.050, -4.866, 0.412},
            {"below_left_out", 0.54830, 0.41412, 0.07209, 2.608, -2.351, 2.380},
            {"bl_in", 0.80791, 0.40842, 0.07209, 2.608, -2.351, 2.380},
            {"bl_looktop", 0.76276, 0.40843, 0.07210, 1.162, 1.275, 1.285},
            {"br_in", 0.79865, -0.43944, -0.33843, 2.369, -2.497, 2.302},
            {"br_lookleft", 0.79863, -0.43943, 0.06152, 0.049, 1.635, -0.009},
            {"br_looktop", 0.79864, -0.43944, 0.06156, 1.292, 1.217, 1.169}
            // Diğer pozları da buraya ekleyebilirsiniz...
        };

        // 2. ADIM: Her poz için ayrı bir publisher oluştur
        for (const auto& pose_data : all_poses_)
        {
            std::string topic_name = "transformed_poses/" + pose_data.name;
            publishers_[pose_data.name] = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PoseTransformerNode::transform_and_publish_all_poses, this));
        
        RCLCPP_INFO(this->get_logger(), "Pose Transformer başlatıldı. %zu adet poz dönüştürülecek.", all_poses_.size());
    }

private:
    void transform_and_publish_all_poses()
    {
        timer_->cancel(); // Timer'ı iptal et
        std::string target_frame = "world";
        std::string source_frame = "ur10e_base";

        RCLCPP_INFO(this->get_logger(), "Dönüşüm işlemi başlıyor: %s -> %s", source_frame.c_str(), target_frame.c_str());

        // 3. ADIM: Vektördeki her bir pozu döngüye al ve dönüştür
        for (const auto& pose_data : all_poses_)
        {
            geometry_msgs::msg::PoseStamped input_pose;
            input_pose.header.stamp = this->get_clock()->now();
            input_pose.header.frame_id = source_frame;

            input_pose.pose.position.x = pose_data.x;
            input_pose.pose.position.y = pose_data.y;
            input_pose.pose.position.z = pose_data.z;

            tf2::Quaternion q;
            q.setRPY(pose_data.rx, pose_data.ry, pose_data.rz);
            input_pose.pose.orientation = tf2::toMsg(q);
            
            RCLCPP_INFO(this->get_logger(), "--- [%s] pozu işleniyor ---", pose_data.name.c_str());

            try
            {
                geometry_msgs::msg::PoseStamped transformed_pose = tf_buffer_->transform(
                    input_pose, target_frame, tf2::durationFromSec(1.0));

                // Sonucu ilgili topic'te yayınla
                publishers_[pose_data.name]->publish(transformed_pose);
                
                RCLCPP_INFO(this->get_logger(), "BAŞARILI. [%s] -> x=%.3f, y=%.3f, z=%.3f",
                    transformed_pose.header.frame_id.c_str(),
                    transformed_pose.pose.position.x,
                    transformed_pose.pose.position.y,
                    transformed_pose.pose.position.z);
            }
            catch (const tf2::TransformException & ex)
            {
                RCLCPP_ERROR(this->get_logger(), "[%s] pozu için transform hatası: %s",
                    pose_data.name.c_str(), ex.what());
            }
        }
    }

    // Member Değişkenleri
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<PoseData> all_poses_;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publishers_;
};

} // namespace pose_transformer

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_transformer::PoseTransformerNode>());
    rclcpp::shutdown();
    return 0;
}