#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_objects/pick_status.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  
  // publish the info if it reached to the goals
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<pick_objects::pick_status>("pick_status", 10);
  
  double pick_x = -6.0;
  double pick_y = 4.0;
  double drop_x = 1.0;
  double drop_y = 0.5;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pick_x;
  goal.target_pose.pose.position.y = pick_y;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending the pickup target");
  ac.sendGoal(goal);
  
  // publish the first topic
  pick_objects::pick_status status;
  status.pick_x = pick_x;
  status.pick_y = pick_y;
  status.drop_x = drop_x;
  status.drop_y = drop_y;
  status.did_reach_first_goal = 0;
  status.did_reach_second_goal = 0;
  pub.publish(status);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the robot picked up the object");
    
    status.did_reach_first_goal = 1;
    pub.publish(status);

    // Wait 5 sec for move_base action server to come up
    ROS_INFO("Waiting 5 seconds at the pickup zone");
    ros::Duration(5.0).sleep();

    goal.target_pose.pose.position.x = drop_x;
    goal.target_pose.pose.position.y = drop_y;
    goal.target_pose.pose.orientation.w = 1.0;
    
    ROS_INFO("Sending the drop off target");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the robot dropped off the object!");
      status.did_reach_second_goal = 1;
      pub.publish(status);
    }
    else {
      ROS_INFO("The base failed to drop off the object for some reason");
    }        
  } else {
    ROS_INFO("The base failed to pick up the object for some reason");
  }

  return 0;
}
