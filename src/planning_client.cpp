#include <ros/ros.h>

/*
 * TODO:
 * - craft path
 *    - specify waypoints
 *      - start
 *      - end
 *      - pickup locations
 *      - machine slots
 *      - drop off locations
 *    - start at one end
 *    - unload item
 *    - drop item
 *    - pick up item
 *    - load item
 *    - return to neutral
 *    - run down rail
 *    - unload item
 *    - drop item
 *    - pick up item
 *    - load item
 *    - return to neutral
 *    - run down rail
 * - call planner
 * - time-parameterize results
 * - display results in RViz - industrial robot simulator instead of tesseract display
 *
 * - build URDF
 *    - example robot
 *    - simple rail
 *    - simple carriage
 *    - collision objects for machines / doors
 * - build SRDF
 *
 * - test at various rail speeds
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_client");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
  return 0;
}
