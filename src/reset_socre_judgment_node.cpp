#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <laser_geometry/laser_geometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>

#include <tf/transform_listener.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/console/time.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

class JudgmentNode
{
public:
    JudgmentNode(ros::NodeHandle nh);

private:
    void mapReceived(const nav_msgs::OccupancyGridConstPtr &msg);
    void laserReceived(const sensor_msgs::LaserScanConstPtr &msg);
    void modelStateReceived(const gazebo_msgs::ModelStates &msg);

    void handleMapMessage(const nav_msgs::OccupancyGrid &msg);

    void requestMap();
    bool getEstimationPose(geometry_msgs::Point *point);
    bool getDiffEstAndGazebo(double *l);

    bool first_map_only_;
    bool first_map_received_;

    std::string gazebo_model_name_;

    pcl::PointCloud<pcl::PointXYZ> map_cloud_;
    sensor_msgs::PointCloud2 ros_map_cloud_;

    tf::TransformListener *tf_listener_;
    laser_geometry::LaserProjection projector_;

    // gazebo_msgs::GetModelState get_model_state_;

    ros::Subscriber laser_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber model_states_sub_;
    ros::Publisher map_cloud_pub_;
    ros::Publisher icp_cloud_pub_;
    ros::Publisher estimate_score_pub_;
    // ros::Publisher test_;
};

JudgmentNode::JudgmentNode(ros::NodeHandle nh)
{
    first_map_only_ = false;
    first_map_received_ = false;
    gazebo_model_name_ = "icart_mini";

    tf_listener_ = new tf::TransformListener(ros::Duration(100.0), true);

    map_sub_ = nh.subscribe("map", 1, &JudgmentNode::mapReceived, this);
    laser_sub_ = nh.subscribe("scan", 10, &JudgmentNode::laserReceived, this);
    model_states_sub_ = nh.subscribe("/gazebo/model_states", 10, &JudgmentNode::modelStateReceived, this);

    map_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 1, true);
    icp_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_cloud",1,true);
    estimate_score_pub_ = nh.advertise<std_msgs::Float64>("estimate_score", 1, true);
    // test_ = nh.advertise<std_msgs::Float64>("test", 1, true);

    // ros::ServiceClient model_state_client_ = nh.serviceClient<gazebo_msgs::GetModelState>("get_model_state");

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

void JudgmentNode::modelStateReceived(const gazebo_msgs::ModelStates &msg){
  int model_index = '\0';
  std_msgs::Float64 data;

  for(int i=0; i<msg.name.size(); i++){
    if(msg.name[i] == gazebo_model_name_){
      model_index = i;
      break;
    }
  }
  if(model_index == '\0'){
    return;
  }
  geometry_msgs::Point model_point = msg.pose[model_index].position;
  geometry_msgs::Point estimate_point;
  if(getEstimationPose(&estimate_point)){
    data.data = pow(pow(model_point.x - estimate_point.x, 2) + pow(model_point.y - estimate_point.y,2 ), 0.5);
    estimate_score_pub_.publish(data);
  }
  ros::Duration(0.1).sleep();
}

bool JudgmentNode::getEstimationPose(geometry_msgs::Point *point){
  double ex, ey;

  try
  {
    tf::StampedTransform trans;
    tf_listener_->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(0.5));
    tf_listener_->lookupTransform("map", "base_link", ros::Time(0), trans);
    ex = trans.getOrigin().x();
    ey = trans.getOrigin().y();
  }
  catch(tf::TransformException &e)
  {
    ROS_WARN("%s", e.what());
    return false;
  }

  point->x = ex;
  point->y = ey;

  return true;
}

bool JudgmentNode::getDiffEstAndGazebo(double *l){
  double gx, gy;
  geometry_msgs::Point point;
  if(!getEstimationPose(&point)){
    return false;
  }
  gazebo_msgs::GetModelState::Request req;
  gazebo_msgs::GetModelState::Response resp;
  req.model_name = "icart_mini";
  if(!ros::service::call("/gazebo/get_model_state", req, resp)){
    return false;
  }
  gx = resp.pose.position.x;
  gy = resp.pose.position.y;

  *l = pow( pow(point.x-gx,2) + pow(point.y-gy,2), 0.5);
  return true;
}

void JudgmentNode::laserReceived(const sensor_msgs::LaserScanConstPtr &msg){
    double real_l;
    std_msgs::Float64 data;
    if(getDiffEstAndGazebo(&real_l)){
      data.data = real_l;
      estimate_score_pub_.publish(data);
    }
    ros::Duration(0.1).sleep();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "reset_score_judge");
    ros::NodeHandle nh;

    JudgmentNode judge(nh);

    return 0;
}
