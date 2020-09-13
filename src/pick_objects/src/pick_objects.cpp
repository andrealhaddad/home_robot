#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <vector>
#include <numeric>
#include <geometry_msgs/Pose2D.h>

using namespace std;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


geometry_msgs::Pose2D generate_geo_goal(move_base_msgs::MoveBaseGoal& g){
  
  geometry_msgs::Pose2D geo_goal;
  geo_goal.x = g.target_pose.pose.position.x;
  geo_goal.y = g.target_pose.pose.position.y;
  geo_goal.theta = g.target_pose.pose.orientation.w;
  
  return geo_goal;
}


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  
  ros::NodeHandle n;
  
  ros::Publisher goal_pub = n.advertise<geometry_msgs::Pose2D>("/target",20);
  
  // positions to visit:
  vector<vector <double> > positions{
    {2.5,2.8, 1.0} , {0.0, 2, -1.0}   };//, {0.13, 0.1, 0.1}
  string message_to_publish;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::Pose2D goal_geo_msg;
  
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();


  for (int i =0; i < positions.size(); i++){

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = positions[i][0];
    goal.target_pose.pose.position.y = positions[i][1];
    goal.target_pose.pose.orientation.w = positions[i][2];
	
    
    goal_geo_msg = generate_geo_goal(goal);
    goal_pub.publish(goal_geo_msg);
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal location #%d ",i);
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      //message_to_publish  = "Horray! Reached position number " + " successfully";
      ROS_INFO("Horray! Reached position number %d" , i);
      if ( i%2 == 0){
        ROS_INFO("Picking up object");
      }
      else {
        ROS_INFO("Dropping off");
      }
      ros::Duration(5.0).sleep();
    }
    else {
      ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
    //ros::Duration(5.0).sleep();

  }
  ros::spin();
  return 0;
}


