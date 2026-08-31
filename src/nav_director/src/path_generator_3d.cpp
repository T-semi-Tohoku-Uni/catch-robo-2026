#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unsupported/Eigen/Splines>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

// 独自メッセージパッケージのヘッダー
#include <catchrobo2026_msgs/srv/waypoint.hpp>
#include <catchrobo2026_msgs/srv/generate_route.hpp>

using WaypointSrv = catchrobo2026_msgs::srv::Waypoint;
using GenRouteSrv = catchrobo2026_msgs::srv::GenerateRoute;

struct Point3D {
    double x, y, z, phi;
};

class PathGenerator3D : public rclcpp::Node {
public:
    PathGenerator3D() : Node("path_generator_3d") {
        // パブリッシャーとサブスクライバーの初期化
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("route", 10);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_orientations", 10);
        
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "current_pose", 10, std::bind(&PathGenerator3D::poseCallback, this, std::placeholders::_1));

        // サービスの初期化
        srv_waypoint_ = this->create_service<WaypointSrv>(
            "waypoint", std::bind(&PathGenerator3D::waypointCallback, this, std::placeholders::_1, std::placeholders::_2));
        srv_gen_route_ = this->create_service<GenRouteSrv>(
            "generate_route", std::bind(&PathGenerator3D::genRouteCallback, this, std::placeholders::_1, std::placeholders::_2));

        // 初期パラメータの設定
        cur_pose_ = {0.0, 0.0, 0.0, 0.0};
        sample_resolution_ = 50; // 各ウェイポイント間の分割数
        
        RCLCPP_INFO(this->get_logger(), "3D Path Generator Initialized (with automatic point densification).");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 現在地を常に更新
        cur_pose_.x = msg->pose.position.x;
        cur_pose_.y = msg->pose.position.y;
        cur_pose_.z = msg->pose.position.z;
        
        // 現在の姿勢(クォータニオン)からヨー角(PHI)を抽出
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        cur_pose_.phi = yaw;
    }

    void waypointCallback(const std::shared_ptr<WaypointSrv::Request> req,
                          std::shared_ptr<WaypointSrv::Response> res) {
        waypoints_.push_back({req->x, req->y, req->z, req->phi});
        res->success = true;
        RCLCPP_INFO(this->get_logger(), "Added 3D Waypoint: [%.2f, %.2f, %.2f, phi: %.2f]", 
                    req->x, req->y, req->z, req->phi);
    }

    void genRouteCallback(const std::shared_ptr<GenRouteSrv::Request> req,
                          std::shared_ptr<GenRouteSrv::Response> res) {
        // 目標地点をリストの最後に追加
        waypoints_.push_back({req->x, req->y, req->z, req->phi});

        // 経路生成元となる点群（現在地 + ウェイポイント群 + 目標地点）
        std::vector<Point3D> route_points;
        route_points.push_back(cur_pose_);
        route_points.insert(route_points.end(), waypoints_.begin(), waypoints_.end());

        // 点数が4点未満の場合、区間を3等分して中間の2点（1/3, 2/3地点）を挿入し、絶対に4点以上にする
        while (route_points.size() < 4) {
            route_points = densifyPoints(route_points);
        }

        // 3DスプラインとSlerpを用いた経路生成
        auto smoothed_path = generate3DSpline(route_points);
        publishPath(smoothed_path);

        // 次の生成に向けてウェイポイントをクリア
        waypoints_.clear();
        res->success = true;
        RCLCPP_INFO(this->get_logger(), "Route generation completed and published.");
    }

    // 隣接する点の間を3等分して2点を作り出し、点数を増やす関数
    std::vector<Point3D> densifyPoints(const std::vector<Point3D>& points) {
        std::vector<Point3D> densified;
        if (points.size() < 2) return points;

        for (size_t i = 0; i < points.size() - 1; ++i) {
            Point3D p0 = points[i];
            Point3D p1 = points[i + 1];

            densified.push_back(p0);

            // 1/3 地点のポイントを生成
            densified.push_back(interpolatePoint(p0, p1, 1.0 / 3.0));

            // 2/3 地点のポイントを生成
            densified.push_back(interpolatePoint(p0, p1, 2.0 / 3.0));
        }
        densified.push_back(points.back());
        return densified;
    }

    // 位置と姿勢（Slerp）を考慮した内分点計算
    Point3D interpolatePoint(const Point3D& p0, const Point3D& p1, double ratio) {
        Point3D p;
        p.x = p0.x + (p1.x - p0.x) * ratio;
        p.y = p0.y + (p1.y - p0.y) * ratio;
        p.z = p0.z + (p1.z - p0.z) * ratio;

        // 姿勢(phi)はクォータニオンのSlerpで自然に補間
        tf2::Quaternion q0, q1;
        q0.setRPY(0, 0, p0.phi);
        q1.setRPY(0, 0, p1.phi);
        tf2::Quaternion q_interp = q0.slerp(q1, ratio);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_interp).getRPY(roll, pitch, yaw);
        p.phi = yaw;

        return p;
    }

    std::vector<Point3D> generate3DSpline(const std::vector<Point3D>& points) {
        using Spline3D = Eigen::Spline<double, 3>;
        Eigen::Matrix<double, 3, Eigen::Dynamic> p_matrix(3, points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            p_matrix(0, i) = points[i].x;
            p_matrix(1, i) = points[i].y;
            p_matrix(2, i) = points[i].z;
        }

        Eigen::RowVectorXd u(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            u(i) = static_cast<double>(i) / static_cast<double>(points.size() - 1);
        }

        // 3次スプライン曲線による位置情報のフィッティング
        const int degree = 3;
        Spline3D spline = Eigen::SplineFitting<Spline3D>::Interpolate(p_matrix, degree, u);

        std::vector<Point3D> smoothed;
        int total_samples = points.size() * sample_resolution_;
        
        for (int i = 0; i <= total_samples; ++i) {
            // tは経路全体の進行度合 (0.0 から 1.0)
            double t = static_cast<double>(i) / total_samples;
            Eigen::Vector3d pv = spline(t);

            // --- 姿勢(PHI)のSlerp補間 ---
            double scaled_t = t * (points.size() - 1);
            int idx = static_cast<int>(std::floor(scaled_t));
            
            if (idx >= static_cast<int>(points.size() - 1)) {
                idx = points.size() - 2;
                scaled_t = points.size() - 1;
            }
            
            double local_t = scaled_t - idx;

            tf2::Quaternion q_start, q_end;
            q_start.setRPY(0, 0, points[idx].phi);
            q_end.setRPY(0, 0, points[idx + 1].phi);

            tf2::Quaternion q_interp = q_start.slerp(q_end, local_t);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q_interp).getRPY(roll, pitch, yaw);

            smoothed.push_back({pv.x(), pv.y(), pv.z(), yaw});
        }
        
        return smoothed;
    }

    void publishPath(const std::vector<Point3D>& path_points) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        visualization_msgs::msg::MarkerArray marker_array;
        
        visualization_msgs::msg::Marker del_marker;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        for (size_t i = 0; i < path_points.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            
            pose.pose.position.x = path_points[i].x;
            pose.pose.position.y = path_points[i].y;
            pose.pose.position.z = path_points[i].z;

            tf2::Quaternion q;
            q.setRPY(0, 0, path_points[i].phi);
            pose.pose.orientation = tf2::toMsg(q);
            
            path_msg.poses.push_back(pose);

            if (i % 10 == 0) {
                visualization_msgs::msg::Marker arrow;
                arrow.header = path_msg.header;
                arrow.ns = "path_orientations";
                arrow.id = static_cast<int>(i);
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.pose = pose.pose;
                
                arrow.scale.x = 0.05;
                arrow.scale.y = 0.01;
                arrow.scale.z = 0.01;

                arrow.color.r = 1.0f;
                arrow.color.g = 0.0f;
                arrow.color.b = 0.0f;
                arrow.color.a = 1.0f;
                
                marker_array.markers.push_back(arrow);
            }
        }
        pub_path_->publish(path_msg);
        pub_marker_->publish(marker_array);
    }

    Point3D cur_pose_;
    std::vector<Point3D> waypoints_;
    int sample_resolution_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Service<WaypointSrv>::SharedPtr srv_waypoint_;
    rclcpp::Service<GenRouteSrv>::SharedPtr srv_gen_route_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathGenerator3D>());
    rclcpp::shutdown();
    return 0;
}