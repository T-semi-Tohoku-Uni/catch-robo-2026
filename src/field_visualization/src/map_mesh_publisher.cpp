#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class MapMeshPublisher : public rclcpp::Node
{
public:
  MapMeshPublisher() : Node("map_mesh_publisher")
  {
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("map_mesh_marker", 10);
    
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapMeshPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // タイムスタンプを統一するために変数に格納
    auto now = this->now();

    // --------------------------------------------------
    // 1. フィールドメッシュのマーカー
    // --------------------------------------------------
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = now;
    marker.ns = "map_mesh";
    marker.id = 0;
    
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.mesh_resource = "package://core/models/field/meshes/catch_field.obj";
    marker.mesh_use_embedded_materials = true;

    marker.pose.position.x = 0.650;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = -0.028 + 0.228;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    publisher_->publish(marker);

    // --------------------------------------------------
    // 2. 原点を示すマーカー（赤い球体）
    // --------------------------------------------------
    auto origin_marker = visualization_msgs::msg::Marker();
    origin_marker.header.frame_id = "map";
    origin_marker.header.stamp = now;
    
    // RViz上で別のオブジェクトとして扱うために ns と id を変更する
    origin_marker.ns = "map_origin";
    origin_marker.id = 1; 
    
    // 形状を球体（SPHERE）に指定
    origin_marker.type = visualization_msgs::msg::Marker::SPHERE;
    origin_marker.action = visualization_msgs::msg::Marker::ADD;

    // 位置は原点 (0, 0, 0)
    origin_marker.pose.position.x = 0.0;
    origin_marker.pose.position.y = 0.0;
    origin_marker.pose.position.z = 0.0;
    origin_marker.pose.orientation.w = 1.0;

    // スケール (球の直径。ここでは x, y, z 全て 0.2m = 20cm に設定)
    origin_marker.scale.x = 0.05;
    origin_marker.scale.y = 0.05;
    origin_marker.scale.z = 0.05;

    // 色を目立つように赤に設定
    origin_marker.color.a = 1.0; 
    origin_marker.color.r = 1.0; // 赤
    origin_marker.color.g = 0.0;
    origin_marker.color.b = 0.0;

    publisher_->publish(origin_marker);
  }
  
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMeshPublisher>());
  rclcpp::shutdown();
  return 0;
}