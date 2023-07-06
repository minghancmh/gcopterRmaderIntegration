/* ----------------------------------------------------------------------------
 * Copyright 2022, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <iostream>
#include <vector>
#include <iomanip>
#include <nlopt.hpp>

#include <Eigen/Dense>
#include <random>
#include "timer.hpp"

#include "solver_nlopt.hpp"

#include <fstream>

#include <eigen3/Eigen/Dense>
#include "octopus_search.hpp"
#include "rmader_types.hpp"
#include "utils.hpp"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "decomp_ros_msgs/PolyhedronArray.h"
#include <decomp_ros_utils/data_ros_utils.h>
#include <std_msgs/String.h>


typedef RMADER_timers::Timer MyTimer;

ConvexHullsOfCurve createStaticObstacle(double x, double y, double z, int num_pol, double bbox_x, double bbox_y,
                                        double bbox_z)
{
  ConvexHullsOfCurve hulls_curve;
  std::vector<Point_3> points;

  points.push_back(Point_3(x - bbox_x / 2.0, y - bbox_y / 2.0, z - bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y - bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y + bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y + bbox_y / 2.0, z - bbox_z / 2.0));

  points.push_back(Point_3(x + bbox_x / 2.0, y + bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y + bbox_y / 2.0, z - bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y - bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y - bbox_y / 2.0, z - bbox_z / 2.0));

  CGAL_Polyhedron_3 hull_interval = cu::convexHullOfPoints(points);

  for (int i = 0; i < num_pol; i++)
  {
    hulls_curve.push_back(hull_interval);  // static obstacle
  }

  return hulls_curve;
}

visualization_msgs::Marker getMarker(Eigen::Vector3d& center, double bbox_x, double bbox_y, double bbox_z, double t_min,
                                     double t_final, double t, int id)
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::CUBE;
  m.header.frame_id = "odom";
  m.header.stamp = ros::Time::now();
  m.ns = "marker_dyn_obs";
  m.action = visualization_msgs::Marker::ADD;
  m.id = id;
  m.color = mu::getColorJet(t, t_min, t_final);
  m.scale.x = bbox_x;
  m.scale.y = bbox_y;  // rviz complains if not
  m.scale.z = bbox_z;  // rviz complains if not
  m.pose.position.x = center.x();
  m.pose.position.y = center.y();
  m.pose.position.z = center.z();
  m.pose.orientation.w = 1.0;
  return m;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "testnlopt");
  ros::NodeHandle nh("~");
  ros::Publisher trajectories_found_pub = 
      nh.advertise<visualization_msgs::MarkerArray>("/trajectories_found", 1000, true);
  ros::Publisher best_trajectory_found_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/best_trajectory_found", 1000, true);
  ros::Publisher convex_hulls_pub = nh.advertise<visualization_msgs::Marker>("/convex_hulls", 1, true);


  std::vector<ros::Publisher> jps_poly_pubs;  // = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_jps", 1, true);
  std::vector<ros::Publisher> traj_obstacle_colored_pubs;
  std::vector<ros::Publisher> best_trajectory_found_intervals_pubs;



  double bbox_x = 1; // original 0.4 for all
  double bbox_y = 1;
  double bbox_z = 5;
  // int num_pol = 4; // original
  int num_pol = 20; // number of obstacles
  int deg_pol = 3;
  int samples_x = 5;  // odd number
  int samples_y = 5;  // odd number
  int samples_z = 5;  // odd number
  // Eigen::Vector3d v_max(20.0, 20.0, 20.0);
  // Eigen::Vector3d a_max(20.0, 20.0, 20.0);
  Eigen::Vector3d v_max(20.0, 20.0, 20.0);
  Eigen::Vector3d a_max(20000.0, 20000.0, 20000.0);
  double dc = 0.002; 

  for (int i = 0; i < num_pol; i++)
  {
    ros::Publisher tmp =
        nh.advertise<visualization_msgs::MarkerArray>("/traj_obstacle_colored_int_" + std::to_string(i), 1, true);
    traj_obstacle_colored_pubs.push_back(tmp);

    ros::Publisher tmp2 = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/poly_jps_int_" + std::to_string(i), 1, true);
    jps_poly_pubs.push_back(tmp2);

    ros::Publisher tmp3 =
        nh.advertise<visualization_msgs::MarkerArray>("/best_trajectory_found_int_" + std::to_string(i), 1, true);
    best_trajectory_found_intervals_pubs.push_back(tmp3);
    
  }

  std::string basis;

  nh.getParam("basis", basis); // gets the basis from the single_obstale.launch file to see how to calculate the trajectory

  std::cout << "Basis= " << basis << std::endl;

  // not used
  double runtime = 0.2;    //[seconds] // originally 0.2
  double goal_size = 0.5;  //[meters] // originally 0.1 (as long as within 0.5m of target, considered goal reach!)
  Eigen::Vector3d q0(-4,0,0); // original -3, 0.5, 1
  Eigen::Vector3d q1 = q0;
  Eigen::Vector3d q2 = q1;  
  Eigen::Vector3d goal(4,0,0); // original 5,0,1
  // double t_min = 0.0;
  // double t_max = t_min + (goal - q0).norm() / (0.8 * v_max(0)); 
  int num_of_obs = 1;  // odd number
  double separation = 0.4;
  int num_of_obs_up = (num_of_obs - 1) / 2.0;
  /////////

  ConvexHullsOfCurves hulls_curves;
  std::vector<visualization_msgs::MarkerArray> ma_vector;

  

  // ConvexHullsOfCurve hulls_curve = createStaticObstacle(0.0, 0.0, bbox_z / 2.0, num_pol, bbox_x, bbox_y, bbox_z);
  // hulls_curves.push_back(hulls_curve);
  // ConvexHullsOfCurve hulls_curve2 = createStaticObstacle(5.0, 5.0, bbox_z / 2.0, num_pol, bbox_x, bbox_y, bbox_z);
  // hulls_curves.push_back(hulls_curve2);


  for (int i = 0; i < ma_vector.size(); i++)
  {
    traj_obstacle_colored_pubs[i].publish(ma_vector[i]);
  }

  std::cout << "hulls_curves.size()= " << hulls_curves.size() << std::endl;

  // mt::ConvexHullsOfCurves_Std hulls_std = cu::vectorGCALPol2vectorStdEigen(hulls_curves);
  std::srand(123);
  for (int i = 0; i < num_pol; i++)
  {
    double x_rand = std::rand()%10 - 5;
    double y_rand = std::rand()%10 - 5;
    // std::cout << "[cgal] Point: " << " x:" << x_rand << " y:" << y_rand << std::endl;

    ConvexHullsOfCurve hc = createStaticObstacle(x_rand ,y_rand , bbox_z / 2.0, num_pol, bbox_x, bbox_y, bbox_z);
    hulls_curves.push_back(hc);

    ConvexHullsOfCurve tmp2;
    ConvexHullsOfCurves tmp;

    tmp2.push_back(hc[i]);
    tmp.push_back(tmp2);

    // convert the obstacles polyhedron arrays
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(cu::vectorGCALPol2vectorJPSPol(tmp));
    poly_msg.header.frame_id = "odom";
    jps_poly_pubs[i].publish(poly_msg);
  }

    mt::ConvexHullsOfCurves_Std hulls_std = cu::vectorGCALPol2vectorStdEigen(hulls_curves);


  //   for (int i = 0; i < num_pol; i++)
  // {
  //   ConvexHullsOfCurve tmp2;
  //   ConvexHullsOfCurves tmp;

  //   tmp2.push_back(hulls_curve[i]);
  //   tmp.push_back(tmp2);

  //   // convert the obstacles polyhedron arrays
  //   decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(cu::vectorGCALPol2vectorJPSPol(tmp));
  //   poly_msg.header.frame_id = "odom";
  //   jps_poly_pubs[0].publish(poly_msg);
  // }




  ms::par_solver parameters;

  // common parameters
  parameters.num_pol = num_pol;
  parameters.deg_pol = deg_pol; // original 3
  parameters.a_star_samp_x = samples_x;
  parameters.a_star_samp_y = samples_y;
  parameters.a_star_samp_z = samples_z;
  parameters.alpha_shrink = 0.9; // original "0.95"
  parameters.a_star_fraction_voxel_size = 0.0; // original 0.5
  parameters.v_max = v_max; // is now 7,7,7, original (20,20,20)
  parameters.a_max = a_max; // is now 400000, 400000, 400000 original (20,20,20)
  parameters.dc = dc; // original 0.01, is now 0.002
  parameters.basis = "MINVO"; // edfited, original "MINVO"
  parameters.a_star_bias = 1.0;
  // parameters.v_max = 20 * Eigen::Vector3d::Ones();
  // parameters.a_max = 20 * Eigen::Vector3d::Ones();
  

  parameters.dist_to_use_straight_guess = 1;
  parameters.weight = 1.0;
  parameters.epsilon_tol_constraints = 0.001; //original 0.001
  parameters.xtol_rel = 0.0000000000001; // originally 0.0000000000001
  parameters.ftol_rel = 0.0000000000001;// originally 0.0000000000001
  parameters.solver = "LD_MMA"; // LD_MMA, LN_NELDERMEAD
  parameters.allow_infeasible_guess = true;
  parameters.Ra =   1e10; // original 4



  SolverNlopt snlopt(parameters);  // snlopt(a,g) a polynomials of degree 3
  snlopt.setMaxRuntimeKappaAndMu(0.25 , 0.5, 0.5); // maxRuntime should be 0.2, kappa and mu should be 0.5 // adjusted to 0.25 maxruntime, generates a more complete path
  mt::state initial_state;
  initial_state.pos = Eigen::Vector3d(-10,10,0);

  mt::state final_state;
  final_state.pos = Eigen::Vector3d(10,-10,0);

  double t_min = 0.0;
  double t_max = t_min + (final_state.pos - initial_state.pos).norm() / (0.8 * parameters.v_max(0)); // original 0.3*parameters.v_max, 0.7 kills the entire vm, do not set to 0.7
  std::cout << "t_min= " << t_min << std::endl;
  std::cout << "t_max= " << t_max << std::endl;
  std::cout << "=========================" << std::endl;
  
  //setting the start and the goal
  snlopt.setInitStateFinalStateInitTFinalT(initial_state, final_state, t_min, t_max);

  //setting the obstacles
  snlopt.setHulls(hulls_std);

  std::cout << "Calling optimize" << std::endl;
  // std::cout << "Code is here" << std::endl;
  bool is_stuck;
  bool is_A_star_failed;
  bool is_q0_fail;
  auto t1 = std::chrono::high_resolution_clock::now();
  bool converged = snlopt.optimize(is_stuck, is_A_star_failed, is_q0_fail);
  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration <double, std::milli> time_delta = t2 - t1;
  std::cout<< termcolor::red << "Time for planner algorithm (RMADER - snlopt.optimize(...)): " << time_delta.count() << "ms" << termcolor::reset << std::endl;
  double time_needed = snlopt.getTimeNeeded();
  std::cout<< "Time_needed (line 241, test_nlopt):"<<time_needed*1000 << "ms" <<std::endl;
  double delta = (t_max - t_min) / num_pol;

  // Recover all the trajectories found and the best trajectory
  // std::vector<mt::trajectory> all_trajs_found;
  // snlopt.getOctopusSolver() -> getAllTrajsFound(all_trajs_found);

  mt::trajectory best_traj_found;
  mt::PieceWisePol pwp_best_traj_found;
  snlopt.getOctopusSolver() -> getBestTrajFound(best_traj_found, pwp_best_traj_found, dc);

  int increm = 2;
  int increm_best = 1;
  double scale = 0.02;
  int j = 0;

//   visualization_msgs::MarkerArray marker_array_all_trajs;
//   for (auto traj : all_trajs_found)
//   {
//     visualization_msgs::MarkerArray marker_array_traj = mu::trajectory2ColoredMarkerArray(
//         traj, v_max.maxCoeff(), increm, "traj" + std::to_string(j), scale, "time", 0, 1);
//     // std::cout << "size of marker_array_traj= " << marker_array_traj.markers.size() << std::endl;
//     for (auto marker : marker_array_traj.markers)
//     {
//       marker_array_all_trajs.markers.push_back(marker);
//     }
//     j++;
//   }

//---> the best trajectory found
  scale = 0.1;
  visualization_msgs::MarkerArray marker_array_best_traj;
  marker_array_best_traj = mu::trajectory2ColoredMarkerArray(best_traj_found, v_max.maxCoeff(), increm_best,
                                                             "traj" + std::to_string(j), scale, "time", 0, 1);

  double entries_per_interval = marker_array_best_traj.markers.size() / num_pol;
  for (int i = 0; i < num_pol; i++)
  {
    std::vector<visualization_msgs::Marker> tmp(                                 /////////
        marker_array_best_traj.markers.begin() + i * entries_per_interval,       /////////
        marker_array_best_traj.markers.begin() + (i + 1) * entries_per_interval  /////////
    );

    visualization_msgs::MarkerArray ma;
    ma.markers = tmp;
    best_trajectory_found_intervals_pubs[i].publish(ma);
  }

  best_trajectory_found_pub.publish(marker_array_best_traj);

  // Get the edges of the convex hulls and publish them
  mt::Edges edges_convex_hulls;
  snlopt.getOctopusSolver() -> getEdgesConvexHulls(edges_convex_hulls);
  convex_hulls_pub.publish(mu::edges2Marker(edges_convex_hulls, mu::color(mu::red_normal)));

  // publish the trajectories (all the other possible trajectories)
  // trajectories_found_pub.publish(marker_array_all_trajs);

  ros::spinOnce();

  ros::spin();

  return 0;
}
// std::ofstream myfile;
// myfile.open("/home/jtorde/Desktop/ws/src/mader/mader/src/solvers/nlopt/example.txt");

// double tmp = 8.0;
//  for (double tmp = 3; tmp < 50; tmp = tmp + 0.05)
// {

// if (converged)
// {
//   myfile << num_pol << ", " << delta << ", " << time_needed << std::endl;
// }
// //  }
// myfile.close();