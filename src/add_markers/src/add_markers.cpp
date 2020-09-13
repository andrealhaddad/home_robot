
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <vector>
#include <numeric>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

// %EndTag(INCLUDES)%


class Create_Marker{
  public:
    Create_Marker();

  private:
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    ros::Subscriber goal_sub;
    ros::Subscriber pose_sub;
    geometry_msgs::Pose2D goal;
    geometry_msgs::PoseWithCovarianceStamped amcl_position;
    visualization_msgs::Marker marker;
    ros::Time time_current;
    double poseAMCLx=0.0, poseAMCLy=0.0, poseAMCLa=0.0;
    double poseGOALx=0.0, poseGOALy=0.0, poseGOALa=0.0;
    bool load_state = false, close_by_flag = false; //false -> robot not loaded, true -> loaded.
    int goal_counter = 0;
  	double distance_threshold = 0.15;
    ////////////////////////
    //add functions:
    void update();
    void amcl_pose_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
    void goal_Callback(const geometry_msgs::Pose2D::ConstPtr&);
  	void update_load_status();
  	void close_by_estimate();
};

 
Create_Marker::Create_Marker() {
  marker_pub = n.advertise<visualization_msgs::Marker>("marker_target", 1);
  goal_sub = n.subscribe("target",1,&Create_Marker::goal_Callback,this);
  pose_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,  &Create_Marker::amcl_pose_Callback, this);


  //ROS_INFO_STREAM("create marker function started");

  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = shape;

  marker.action = visualization_msgs::Marker::ADD; /// or delete

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
//   marker.pose.position.x = 2.0;
//   marker.pose.position.y = 2.0;
//   marker.pose.position.z = 0.0;
//   marker.pose.orientation.z = 0.0;
//   marker.action = visualization_msgs::Marker::ADD;
//   marker.pose.orientation.x = 0.0;
//   marker.pose.orientation.y = 0.0;
  //marker.pose.orientation.w = 0.0;


  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.4;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.3f;
  marker.color.g = 0.5f;
  marker.color.b = 0.7f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  ROS_INFO_STREAM("published");
  shape = visualization_msgs::Marker::CUBE;

  this->update();
}



void Create_Marker::amcl_pose_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL){
  poseAMCLx = msgAMCL->pose.pose.position.x;
  poseAMCLy = msgAMCL->pose.pose.position.y;
  poseAMCLa = msgAMCL->pose.pose.orientation.w;
  //ROS_INFO("I am here");

  this->update();
}

void Create_Marker::goal_Callback(const geometry_msgs::Pose2D::ConstPtr& msgGOAL){
  poseGOALx = msgGOAL->x;
  poseGOALy = msgGOAL->y;
  poseGOALa = msgGOAL->theta;
  ROS_INFO("GOAL sent %f , %f, %f", poseGOALx, poseGOALy, poseGOALa);
  goal_counter += 1;
  ROS_INFO("GOAL COUNTER %d", goal_counter);
  this->update();
}

void Create_Marker::update_load_status(){
  if (goal_counter%2 == 0){
    load_state = true;
    ROS_INFO("Load status: true");
  }
  else{
    load_state = false;
    ROS_INFO("Load status: false");
  }
}

void Create_Marker::close_by_estimate(){
  //calculate distance to marker
  double dist = sqrt( pow(poseAMCLx - poseGOALx, 2) + pow(poseAMCLy - poseGOALy, 2) );

  if (dist < distance_threshold){
    close_by_flag = true;
  }
  else{
    close_by_flag = false;
  } 
  ROS_INFO("Close by: dist-> %f , close by-> %d", dist, close_by_flag );
}

void Create_Marker::update(){
  
  //Marker location is always at goal. Its appearance change.
  marker.pose.position.x = poseGOALx;
  marker.pose.position.y = poseGOALy;
  marker.pose.position.z = 0.2;
  marker.pose.orientation.z = poseGOALa;

  //Far from object
  if (close_by_flag == false){
    // Check if load is empty or not
    if (load_state == false){
      //empty load and far away then show marker
      marker.action = visualization_msgs::Marker::ADD;}
    else{
      marker.action = visualization_msgs::Marker::DELETE;
    }
  }
  
  //Close to object
  if (close_by_flag == true){
    // Check if load is empty or not
    if (load_state == false){
      //of empty load and far away then NO marker
      marker.action = visualization_msgs::Marker::DELETE;
      //ros::Duration(5).sleep();
    }
    else{
      marker.action = visualization_msgs::Marker::ADD;
    }
  }

  if (ros::ok()){
    marker_pub.publish(marker);
  }
  
  Create_Marker::update_load_status();
  Create_Marker::close_by_estimate();

}
// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ROS_INFO_STREAM("Starting class");
  Create_Marker create_marker;
  ros::spin();
  //r.sleep();
}