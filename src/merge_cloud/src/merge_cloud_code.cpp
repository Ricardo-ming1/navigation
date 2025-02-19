#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

using sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
using livox_ros_driver2::msg::CustomMsg;
using CustomMsgConstPtr = livox_ros_driver2::msg::CustomMsg::ConstSharedPtr;

class SubscribeAndPublish : public rclcpp::Node
{
public:
    SubscribeAndPublish() : Node("point_cloud_merge")
    {
        // 创建发布者
        pub1 = this->create_publisher<CustomMsg>("/livox/lidar", 10);
        pub2 = this->create_publisher<PointCloud2>("/livox/lidar/pointcloud", 10);

        // 创建订阅者
        sub_left = std::make_shared<message_filters::Subscriber<CustomMsg>>(this, "/livox/lidar_192_168_1_187");
        sub_right = std::make_shared<message_filters::Subscriber<CustomMsg>>(this, "/livox/lidar_192_168_1_3");
        
        sub_pclleft = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/livox/lidar/pointcloud_192_168_1_187");
        sub_pclright = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/livox/lidar/pointcloud_192_168_1_3");
        
        // 定义同步策略
        using CustomSyncPolicy = message_filters::sync_policies::ApproximateTime<CustomMsg, CustomMsg>;
        sync_custom = std::make_shared<message_filters::Synchronizer<CustomSyncPolicy>>(CustomSyncPolicy(10), *sub_left, *sub_right);
        sync_custom->registerCallback(std::bind(&SubscribeAndPublish::customCallBack, this, std::placeholders::_1, std::placeholders::_2));

        using PointCloudSyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;
        sync_pcl = std::make_shared<message_filters::Synchronizer<PointCloudSyncPolicy>>(PointCloudSyncPolicy(10), *sub_pclleft, *sub_pclright);
        sync_pcl->registerCallback(std::bind(&SubscribeAndPublish::pclCallBack, this, std::placeholders::_1, std::placeholders::_2));
    }
private:
    void customCallBack(const CustomMsgConstPtr& lidar_left, const CustomMsgConstPtr& lidar_right)
    {
        
        PointCloudXYZI pointCloud_left, pointCloud_left_out;
        PointCloudXYZI pointCloud_right, pointCloud_right_out;
        PointCloudXYZI finalPointCloud;

        //将CustmMsg 转换 为pointCloud2
        convert2PointCloud2(lidar_left, pointCloud_left);
		convert2PointCloud2(lidar_right, pointCloud_right);

        // 定义变换矩阵
        Eigen::Matrix4f transform2Left = Eigen::Matrix4f::Identity();
        transform2Left(0, 0) = 1;
        transform2Left(0, 1) = 0;
        transform2Left(0, 2) = 0;
        transform2Left(0, 3) = 0.14;

        transform2Left(1, 0) = 0;
        transform2Left(1, 1) = 1;
        transform2Left(1, 2) = 0;
        transform2Left(1, 3) = 0;

        transform2Left(2, 0) = 0;
        transform2Left(2, 1) = 0;
        transform2Left(2, 2) = 1;
        transform2Left(2, 3) = 0;

        Eigen::Matrix4f transform2Right = Eigen::Matrix4f::Identity();
        transform2Right(0, 0) = -1;
        transform2Right(0, 1) = 0;
        transform2Right(0, 2) = 0;
        transform2Right(0, 3) = -0.38;

        transform2Right(1, 0) = 0;
        transform2Right(1, 1) = -1;
        transform2Right(1, 2) = 0;
        transform2Right(1, 3) = 0;

        transform2Right(2, 0) = 0;
        transform2Right(2, 1) = 0;
        transform2Right(2, 2) = 1;
        transform2Right(2, 3) = 0;

        // 对点云进行变换
        pcl::transformPointCloud(pointCloud_left, pointCloud_left_out, transform2Left);
        pcl::transformPointCloud(pointCloud_right, pointCloud_right_out, transform2Right);

        // 合并点云
        finalPointCloud = pointCloud_left_out + pointCloud_right_out;

        // 将PointCloud2点云转换为 Livox 自定义的 CustomMsg

        // ???
        auto finalMsg = std::make_shared<CustomMsg>();
        finalMsg->header=lidar_left->header;             //使用左云的时间戳
        finalMsg->timebase = lidar_left->timebase;
		finalMsg->point_num = finalPointCloud.size();
        finalMsg->lidar_id = lidar_left->lidar_id;

        for(unsigned int i = 0; i < finalMsg->point_num; i++)
		{
			livox_ros_driver2::msg::CustomPoint customPoint;
            customPoint.x = finalPointCloud[i].x;
            customPoint.y = finalPointCloud[i].y;
            customPoint.z = finalPointCloud[i].z;
            customPoint.reflectivity = finalPointCloud[i].intensity;
            customPoint.offset_time =finalPointCloud[i].curvature *float(1000000);  // 时间偏移（微秒）
            finalMsg->points.push_back(customPoint);
		}

        // 发布合并后的点云
        pub1->publish(*finalMsg);
    }
void pclCallBack(const PointCloud2ConstPtr& pcl_left, const PointCloud2ConstPtr& pcl_right)
    {
        PointCloudXYZI pointCloud_left, pointCloud_left_out;
        PointCloudXYZI pointCloud_right, pointCloud_right_out;
        PointCloudXYZI finalPointCloud;

        // 将 PointCloud2 转换为 PCL 点云
        pcl::fromROSMsg(*pcl_left, pointCloud_left);
        pcl::fromROSMsg(*pcl_right, pointCloud_right);

        // 定义变换矩阵
        Eigen::Matrix4f transform2Left = Eigen::Matrix4f::Identity();
        transform2Left(0, 3) = 0.14;  // X 方向平移

        Eigen::Matrix4f transform2Right = Eigen::Matrix4f::Identity();
        transform2Right(0, 0) = -1;   // X 方向翻转
        transform2Right(1, 1) = -1;   // Y 方向翻转
        transform2Right(0, 3) = -0.38; // X 方向平移

        // 对点云进行变换
        pcl::transformPointCloud(pointCloud_left, pointCloud_left_out, transform2Left);
        pcl::transformPointCloud(pointCloud_right, pointCloud_right_out, transform2Right);

        // 合并点云
        finalPointCloud = pointCloud_left_out + pointCloud_right_out;

        // 将 PCL 点云转换为 PointCloud2 并发布
        PointCloud2 pcl_msg;
        pcl::toROSMsg(finalPointCloud, pcl_msg);
        pcl_msg.header = pcl_left->header;  // 使用左云的时间戳
        pub2->publish(pcl_msg);
    }

void convert2PointCloud2(const CustomMsgConstPtr& lidarMsg, PointCloudXYZI& pclPointCloud ){
		for(unsigned int i = 0; i < lidarMsg->point_num; i++)
		{
			PointType point;
			point.x = lidarMsg->points[i].x;
			point.y = lidarMsg->points[i].y;
			point.z = lidarMsg->points[i].z;
			point.intensity = lidarMsg->points[i].reflectivity;
			point.curvature = lidarMsg->points[i].offset_time / float(1000000);

			pclPointCloud.push_back(point);
		}
	}
    
    // 成员变量
    rclcpp::Publisher<CustomMsg>::SharedPtr pub1;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub2;
    std::shared_ptr<message_filters::Subscriber<CustomMsg>> sub_left;
    std::shared_ptr<message_filters::Subscriber<CustomMsg>> sub_right;
    std::shared_ptr<message_filters::Subscriber<PointCloud2>> sub_pclleft;
    std::shared_ptr<message_filters::Subscriber<PointCloud2>> sub_pclright;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<CustomMsg, CustomMsg>>> sync_custom;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>>> sync_pcl;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscribeAndPublish>());
    rclcpp::shutdown();
    return 0;
}