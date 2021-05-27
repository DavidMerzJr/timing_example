#include <cmath>
#include <iostream>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

double calculateTime(
    const std::vector<double>& start,
    const std::vector<double>& end,
    bool fast = true)
{
  const std::vector<double> FAST =
  {
    2.5,
    2.0,
    2.0,
    3.9,
    4.3,
    5.75,
    2.0
  };

  const std::vector<double> SLOW =
  {
    0.5,
    0.4,
    0.4,
    0.8,
    0.8,
    1.0,
    0.4
  };

  std::vector<double> limits;
  if (fast)
    limits = FAST;
  else
    limits = SLOW;

  double time = 0.0;
  for (std::size_t i = 0; i < 7; ++i)
  {
    double t = std::fabs((end[i] - start[i]) / limits[i]);
    if (t > time)
      time = t;
  }
  return time;
}

void interpolatePoints(
    const std::vector<double>& start,
    const std::vector<double>& end,
    double time,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory,
    bool fast = true)
{
  trajectory_msgs::JointTrajectoryPoint point;
  int num_points = 10;
  time = calculateTime(start, end, fast);

  for (int i = 1; i <= num_points; ++i)
  {
    point.positions =
    {
      start[0] + i / static_cast<double>(num_points) * (end[0] - start[0]),
      start[1] + i / static_cast<double>(num_points) * (end[1] - start[1]),
      start[2] + i / static_cast<double>(num_points) * (end[2] - start[2]),
      start[3] + i / static_cast<double>(num_points) * (end[3] - start[3]),
      start[4] + i / static_cast<double>(num_points) * (end[4] - start[4]),
      start[5] + i / static_cast<double>(num_points) * (end[5] - start[5]),
      start[6] + i / static_cast<double>(num_points) * (end[6] - start[6])
    };
    point.time_from_start = trajectory.back().time_from_start + ros::Duration(time / num_points);
    trajectory.push_back(point);
  }

  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_client");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> industrial_robot_client ("/joint_trajectory_action", false);
  industrial_robot_client.waitForServer(ros::Duration(1.0));

  const std::vector<std::string> JOINT_NAMES =
  {
    "robot_joint_1",
    "robot_joint_2",
    "robot_joint_3",
    "robot_joint_4",
    "robot_joint_5",
    "robot_joint_6",
    "rail_actuation_joint"
  };

  const std::vector<double> FAST =
  {
    2.5,
    2.0,
    2.0,
    3.9,
    4.3,
    5.75,
    2.0
  };

  const std::vector<double> SLOW =
  {
    0.5,
    0.4,
    0.4,
    0.8,
    0.8,
    1.0,
    0.4
  };

  const std::vector<double> REST_1 =
  {
    0.0,
    -0.88,
    1.13,
    0.0,
    0.0,
    0.0,
    0.0
  };

  const std::vector<double> REST_2 =
  {
    0.0,
    -0.88,
    1.13,
    0.0,
    0.0,
    0.0,
    5.0
  };

  const std::vector<double> POUNCE =
  {
    -1.57,
    -0.88,
    1.13,
    0.0,
    -0.2,
    0.0,
    5.0
  };

  const std::vector<double> POUNCE_2 =
  {
    0.75,
    -0.2,
    0.75,
    0.0,
    1.0,
    0.0,
    5.0
  };

  const std::vector<double> MACHINE_POINT =
  {
    -1.57,
    0.15,
    0.45,
    -1.6,
    -1.6,
    0.2,
    5.0
  };

  const std::vector<double> PICK_POINT =
  {
    0.6,
    0.65,
    0.1,
    0.0,
    0.8,
    -2.6,
    5.0
  };

  const std::vector<double> PLACE_POINT =
  {
    1.1,
    0.0,
    1.0,
    0.0,
    0.55,
    -2.0,
    5.0
  };

  // Make the good path
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = JOINT_NAMES;
  trajectory_msgs::JointTrajectoryPoint point;

  // Start position
  point.positions = REST_1;
  point.time_from_start = ros::Duration(0.0);
  goal.trajectory.points.push_back(point);

  // First point - go to other lathe
  interpolatePoints(REST_1, REST_2, 5.0, goal.trajectory.points);

  // Go to the pounce point
  interpolatePoints(REST_2, POUNCE, 0.75, goal.trajectory.points);

  // Grab part from machine
  interpolatePoints(POUNCE, MACHINE_POINT, 1.5, goal.trajectory.points, false);

  // Go back to the pounce point
  interpolatePoints(MACHINE_POINT, POUNCE, 1.5, goal.trajectory.points, false);

  // Go to the alternate pounce point
  interpolatePoints(POUNCE, POUNCE_2, 1.5, goal.trajectory.points);

  // Place part in complete bin
  interpolatePoints(POUNCE_2, PLACE_POINT, 1.5, goal.trajectory.points, false);

  // Go to the alternate pounce point
  interpolatePoints(PLACE_POINT, POUNCE_2, 1.5, goal.trajectory.points, false);

  // Pick up new part
  interpolatePoints(POUNCE_2, PICK_POINT, 1.5, goal.trajectory.points, false);

  // Go back to the alternate pounce point
  interpolatePoints(PICK_POINT, POUNCE_2, 1.5, goal.trajectory.points, false);

  // Go to proper pounce point
  interpolatePoints(POUNCE_2, POUNCE, 1.5, goal.trajectory.points);

  // Place new part in machine
  interpolatePoints(POUNCE, MACHINE_POINT, 1.5, goal.trajectory.points, false);

  // Go to the pounce point
  interpolatePoints(MACHINE_POINT, POUNCE, 1.5, goal.trajectory.points, false);

  // Go back to rest for transit
  interpolatePoints(POUNCE, REST_2, 0.75, goal.trajectory.points);

  // And run back down the rail
  interpolatePoints(REST_2, REST_1, 5.0, goal.trajectory.points);

  ROS_INFO("Awaiting Command!");
  // Wait for the recording to start
  std::string pause;
  std::cin >> pause;

  // And finally, send the trajectory over
  ROS_INFO("0s");
  industrial_robot_client.sendGoal(goal);
  if (industrial_robot_client.waitForResult(ros::Duration(30.0)))
  {
    ROS_ERROR("OH NO EXECUTION ERROR");
  }

  ROS_INFO("29s!");
  return 0;
}
