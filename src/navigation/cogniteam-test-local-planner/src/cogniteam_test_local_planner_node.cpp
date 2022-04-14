/*
 * waypoint_driver_node.cpp
 *
 *  Created on: Jan 1, 2022
 *      Author: yakirhuri
 */

#include <vector>
#include <string>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_core/base_local_planner.h>


#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>

#include <nav_msgs/Path.h>

using namespace std;

class CogniteamTest
{
public:
  CogniteamTest(tf2_ros::Buffer& tf)
	: tf_(tf)
	, planner_costmap_ros_(NULL)
	, controller_costmap_ros_(NULL)
	, blp_loader_("nav_core", "nav_core::BaseLocalPlanner")
  {

    ros::NodeHandle nh;

	// create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying
	// map
	controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
	controller_costmap_ros_->pause();

	planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
	planner_costmap_ros_->pause();

	vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	pathPub_ = nh.advertise<nav_msgs::Path>(
			"/polygon_path", 1, false);	

	readWaypoints(2);

	try
	{
	  tc_ = blp_loader_.createInstance(local_planner);
	  ROS_INFO("Created local_planner %s", local_planner.c_str());
	  tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
	}
	catch (const pluginlib::PluginlibException& ex)
	{
	  ROS_FATAL(
		  "Failed to create the %s planner, are you sure it is properly registered and that the containing library is "
		  "built? Exception: %s",
		  local_planner.c_str(), ex.what());
	  exit(1);
	}

	drive();
  }

  ~CogniteamTest()
  {
	tc_.reset();

  	//  free(blp_loader_);

	free(planner_costmap_ros_);
	free(controller_costmap_ros_);

	// // set up plan triple buffer
	delete[] planner_plan_;
	delete[] latest_plan_;
  }

  void drive()
  {	


	tc_->setPlan(*&controller_plan_);

	publishPath();

	//ros::Rate rate(0.01); // ROS Rate at 5Hz
	while (ros::ok())
	{
	  // check to see if we've reached our goal
	  if (tc_->isGoalReached())
	  {
		ROS_DEBUG_NAMED("move_base", "Goal reached!");
		return;
	  }

	  geometry_msgs::Twist cmd_vel;

	  if( tc_->computeVelocityCommands(cmd_vel) ){

		//cerr<<"Got a valid command from the local planner "<<endl;
		// make sure that we send the velocity command to the base
		
		vel_pub_.publish(cmd_vel);


	  }	
	  

	   //rate.sleep(); 	
	   ros::spinOnce();

	}
  }

private:

	void publishPath(){

		nav_msgs::Path msgMsg;
		msgMsg.header.frame_id ="map";
		msgMsg.header.stamp = ros::Time::now();
		msgMsg.poses = controller_plan_;

		pathPub_.publish(msgMsg);

		ros::Duration(1).sleep();

	}

	bool readWaypoints(int skip = 1) {
	
		ros::NodeHandle nodePrivate("~");

		vector<string> waypointsList;

		updateRobotLocation();

		if (nodePrivate.getParam("waypoints", waypointsList)) {

			geometry_msgs::PoseStamped pose;

			int line = -1;
			
			for(auto waypointString : waypointsList) {
				double heading = 0;

				line++;	

				// if (controller_plan_.size() > 7 ){
				// 		break;
				// }			

				if( line % skip != 0 ){

						
					continue;
				}
				cerr<< waypointString<<endl;
				auto parsedValues = sscanf(waypointString.c_str(), "%lf,%lf,%lf",
						&pose.pose.position.x,
						&pose.pose.position.y,
						&heading);

				pose.header.frame_id = "map";
				pose.header.stamp = ros::Time(0);

				pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
				

				pose.pose.position.x += robotPose_.pose.position.x;
				pose.pose.position.y += robotPose_.pose.position.y;

				// cerr<<"11111111111111111111111111 "<<endl;
				controller_plan_.push_back(pose);
				// cerr<<"2222222222222222 "<<endl;


				if (parsedValues < 3) {
					ROS_ERROR("Failed to parse a waypoint (line %i)", line);
					return false;
				}

				
				
			}

		} else {


			ROS_ERROR("Error: waypoints parameter does not exists or empty");
			
			return false;
		}

	

		return true;
	}

	bool updateRobotLocation() {

        tf::StampedTransform transform;

        try
        {
            //get current robot pose
            tfListener_.lookupTransform("map", "base_link",
                                        ros::Time(0), transform);

            robotPose_.pose.position.x = transform.getOrigin().x();
            robotPose_.pose.position.y = transform.getOrigin().y();
            robotPose_.pose.position.z = 0;
            robotPose_.pose.orientation.x = transform.getRotation().x();
            robotPose_.pose.orientation.y = transform.getRotation().y();
            robotPose_.pose.orientation.z = transform.getRotation().z();
            robotPose_.pose.orientation.w = transform.getRotation().w();



            return true;
        }

        catch (...)
        {
            cerr << " error between map to base_link" << endl;
            return false;
        }
    }


private:
  boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;

  geometry_msgs::PoseStamped robotPose_;

  tf::TransformListener tfListener_;

  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;

  tf2_ros::Buffer& tf_;

  //string local_planner = "dwa_local_planner/DWAPlannerROS";	 

  string local_planner = "teb_local_planner/TebLocalPlannerROS";	 


  costmap_2d::Costmap2DROS *planner_costmap_ros_, *controller_costmap_ros_;

  // set up plan triple buffer
  std::vector<geometry_msgs::PoseStamped>* planner_plan_;
  std::vector<geometry_msgs::PoseStamped>* latest_plan_;
  std::vector<geometry_msgs::PoseStamped> controller_plan_;

   ros::Publisher vel_pub_;

   ros::Publisher  pathPub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cogniteam-test-local-planner");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  CogniteamTest test(buffer);

  ros::spin();

	

  return 0;
}