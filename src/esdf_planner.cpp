#include <ros/ros.h>

#include <plan_env/edt_environment.h>

using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "hitlfast_planner");
  ros::NodeHandle nh("~");

  EDTEnvironment::Ptr edt_environment_;
  SDFMap::Ptr sdf_map_;

  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}

