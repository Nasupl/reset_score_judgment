#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Header.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

class JudgmentNode
{
public:
    JudgmentNode(ros::NodeHandle nh);

private:
    void mapReceived(const nav_msgs::OccupancyGridConstPtr &msg);
    void laserReceived(const sensor_msgs::LaserScanConstPtr &msg);

    void handleMapMessage(const nav_msgs::OccupancyGrid &msg);

    void requestMap();

    bool first_map_only_;
    bool first_map_received_;

    pcl::PointCloud<pcl::PointXYZ> map_cloud_;
    sensor_msgs::PointCloud2 ros_map_cloud_;

    tf::TransformListener *tf_listener_;
    laser_geometry::LaserProjection projector_;

    ros::Subscriber laser_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher map_cloud_pub_;
};

JudgmentNode::JudgmentNode(ros::NodeHandle nh)
{
    first_map_only_ = false;
    first_map_received_ = false;
    tf_listener_ = new tf::TransformListener();

    map_sub_ = nh.subscribe("map", 1, &JudgmentNode::mapReceived, this);
    laser_sub_ = nh.subscribe("scan", 10, &JudgmentNode::laserReceived, this);

    map_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 1, true);

    ros::spin();
}


void JudgmentNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  ROS_INFO_STREAM("received map");
  if( first_map_only_ && first_map_received_ ) {
    return;
  }

  handleMapMessage( *msg );

  first_map_received_ = true;
}

void JudgmentNode::requestMap()
{
  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  handleMapMessage( resp.map );

  first_map_received_ = true;
}

void JudgmentNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg){
    // is_denseをfalseにしたポイントクラウドを生成する(is_dense=FalseだとNaNの入った点群を生成できる)
    // amclの原点合わせに従って原点を0,0,0に配置する?
    // 占有されてるセルの場所に点を配置する？
    ROS_INFO_STREAM("handled");

    pcl::PointCloud<pcl::PointXYZ> map_cloud;

    map_cloud.width = msg.info.width;
    map_cloud.height = msg.info.height;
    map_cloud.is_dense = false;
    map_cloud.points.resize(map_cloud.width * map_cloud.height);

    float resolution = msg.info.resolution;
    int orig_cell_x = msg.info.origin.position.x/resolution;
    int orig_cell_y = msg.info.origin.position.y/resolution;
    ROS_INFO_STREAM(msg.info.origin.position.x << "," << resolution << "," << orig_cell_x);

    for(int i=0; i<msg.info.width*msg.info.height; i++){
        if(msg.data[i] == 100){
            int i_x = i/map_cloud.width;
            int i_y = i%map_cloud.width;
            map_cloud.points[i].y = i_x * resolution + orig_cell_x * resolution;
            map_cloud.points[i].x = i_y * resolution + orig_cell_y * resolution;
            map_cloud.points[i].z = 0.0;
        }
        else{
            map_cloud.points[i].x = std::numeric_limits<float>::quiet_NaN();
            map_cloud.points[i].y = std::numeric_limits<float>::quiet_NaN();
            map_cloud.points[i].z = std::numeric_limits<float>::quiet_NaN();
        }
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(map_cloud, map_cloud_, indices);

    ROS_INFO_STREAM(map_cloud_.points[0].x << "," << map_cloud_.points[0].y);

    map_cloud_.header.frame_id = msg.header.frame_id;
    map_cloud_.header.stamp = msg.header.stamp.toNSec()/1e3;

    pcl::toROSMsg(map_cloud_, ros_map_cloud_);
    map_cloud_pub_.publish(ros_map_cloud_);

    ROS_INFO_STREAM("converted");
    // std::string str = "test.pcd";
    // pcl::io::savePCDFileASCII(str, map_cloud);
}

void JudgmentNode::laserReceived(const sensor_msgs::LaserScanConstPtr &msg){
    ROS_INFO_STREAM("received laser");
    if(map_cloud_.points.size() == 0){
        ROS_WARN("map cloud is empty");
        return;
    }

    map_cloud_.header.stamp = ros::Time(0).toNSec()/1e3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    try{
        pcl_ros::transformPointCloud("/base_link", map_cloud_, *transformed_map_cloud, *tf_listener_);
    }
    catch (tf::TransformException e){
        ROS_ERROR("ERROR: %s",e.what());
    }

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(transformed_map_cloud);
    pass_x.setFilterLimitsNegative(false);
    pass_x.setFilterFieldName(std::string("x"));
    pass_x.setFilterLimits(0, 30);
    pass_x.filter(*transformed_map_cloud);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(transformed_map_cloud);
    pass_y.setFilterLimitsNegative(false);
    pass_y.setFilterFieldName(std::string("y"));
    pass_y.setFilterLimits(-30, 30);
    pass_y.filter(*transformed_map_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr passthroughed_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    try{
        pcl_ros::transformPointCloud("/map", *transformed_map_cloud, *passthroughed_map_cloud, *tf_listener_);
    }
    catch (tf::TransformException e){
        ROS_ERROR("ERROR: %s",e.what());
    }

    sensor_msgs::PointCloud2 scan_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_scan_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    try{
        tf_listener_->waitForTransform(msg->header.frame_id, "/map", ros::Time(0),ros::Duration(1.0));
    }
    catch (tf::TransformException e){
        ROS_ERROR("ERROR: %s",e.what());
        return;
    }

    try{
        projector_.transformLaserScanToPointCloud("/map", *msg, scan_cloud, *tf_listener_);
    }
    catch (tf::TransformException e){
        ROS_ERROR("ERROR: %s",e.what());
        return;
    }
    pcl::fromROSMsg(scan_cloud, *pcl_scan_cloud);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(passthroughed_map_cloud);
    icp.setInputTarget(pcl_scan_cloud);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    ROS_INFO_STREAM("transformed cloud publish");
    pcl::toROSMsg(*passthroughed_map_cloud, ros_map_cloud_);
    map_cloud_pub_.publish(ros_map_cloud_);

    ros::Duration(0.1).sleep();

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "reset_score_judge");
    ros::NodeHandle nh;

    JudgmentNode judge(nh);

    return 0;
}
