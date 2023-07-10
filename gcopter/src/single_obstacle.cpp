/* ----------------------------------------------------------------------------
 * Copyright 2022, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <eigen3/Eigen/Dense>
// #include "octopus_search.hpp"
#include "termcolor.hpp"
#include "cgal_utils.hpp"
#include "octopus_search.hpp"
#include "utils.hpp"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "decomp_ros_msgs/PolyhedronArray.h"
#include <decomp_ros_utils/data_ros_utils.h>  //For DecompROS::polyhedron_array_to_ros

#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
#include <chrono>

struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
    }
};

class GlobalPlanner
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    double trajStamp;

public:
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh)
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        mapSub = nh.subscribe(config.mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        // subscribes to the nav goal
        // config.targetTopic is set to /move_base_simple/goal, which allows targetSub to subscribe to 2D nav goal from rviz
        targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    }

    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    inline void plan()
    {   
        // std::cout<<"Planning"<<std::endl;
        Eigen::Vector3d start(-10,10,0);
        Eigen::Vector3d end(10,-10,0);
        startGoal.clear();
        startGoal.emplace_back(start);
        startGoal.emplace_back(end); 
        if (startGoal.size() == 2)
        {   
            std::vector<Eigen::Vector3d> route;
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.01,
                                                   route);
            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            voxelMap.getSurf(pc);

            sfc_gen::convexCover(route,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0,
                                 3.0,
                                 hPolys);
            sfc_gen::shortCut(hPolys);

            if (route.size() > 1)
            {
                visualizer.visualizePolytope(hPolys);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();

                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return;
                }

                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return;
                }

                if (traj.getPieceNum() > 0)
                {
                    trajStamp = ros::Time::now().toSec();
                    // std::cout << "visualizing now" << std::endl;

                    visualizer.visualize(traj, route);
                }
            }
        }
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // std::cout << "targetCallBack" << std::endl;
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }
            const double zGoal = config.mapBound[4] + config.dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            // const Eigen::Vector3d goal(-4,0,0);
            if (voxelMap.query(goal) == 0)
            {
                visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
                startGoal.emplace_back(goal);
            }
            else
            {
                ROS_WARN("Infeasible Position Selected !!!\n");
            }

            auto t1 = std::chrono::high_resolution_clock::now();
            plan();
            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration <double, std::milli> time_delta = t2 - t1;

            std::cout<< termcolor::red << "Time for planner algorithm (GCOPTER): " << time_delta.count() << "ms" << termcolor::reset << std::endl;
        }
        return;
    }

    inline void process()
    {
        // std::cout << "Process" << std::endl;
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        if (traj.getPieceNum() > 0)
        {   
            // std::cout << "process traj" << std::endl;
            const double delta = ros::Time::now().toSec() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                double thr;
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;

                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                0.0, 0.0,
                                thr, quat, omg);
                double speed = traj.getVel(delta).norm();
                double bodyratemag = omg.norm();
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
                std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;
                thrMsg.data = thr;
                tiltMsg.data = tiltangle;
                bdrMsg.data = bodyratemag;
                visualizer.speedPub.publish(speedMsg);
                visualizer.thrPub.publish(thrMsg);
                visualizer.tiltPub.publish(tiltMsg);
                visualizer.bdrPub.publish(bdrMsg);

                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);
            }
        }
    }
};
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
  ros::init(argc, argv, "testSingleObstacle");
  // ros::NodeHandle nh("~");
  ros::NodeHandle nh_;
  GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);

  double bbox_x = 1; // original 0.4 for all
  double bbox_y = 1;
  double bbox_z = 2;
  int num_pol = 7;
  int deg_pol = 3;

  double dc = 0.002;  // Simply used for visualization

  int num_of_obs = 1;  // odd number
  double separation = 0.4;

  int num_of_obs_up = (num_of_obs - 1) / 2.0;

//   ConvexHullsOfCurves hulls_curves;
//   ConvexHullsOfCurve hulls_curve = createStaticObstacle(0.0, 0.0, bbox_z / 2.0, num_pol, bbox_x, bbox_y, bbox_z);
//   hulls_curves.push_back(hulls_curve);


//   mt::ConvexHullsOfCurves_Std hulls_std = cu::vectorGCALPol2vectorStdEigen(hulls_curves);

    // std::vector<ros::Publisher> jps_poly_pubs;  // = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_jps", 1, true);
//   std::vector<ros::Publisher> traj_obstacle_colored_pubs;
//   std::vector<ros::Publisher> best_trajectory_found_intervals_pubs;



//   for (int i = 0; i < num_pol; i++)
//   {
//     ros::Publisher tmp =
//         nh_.advertise<visualization_msgs::MarkerArray>("/traj_obstacle_colored_int_" + std::to_string(i), 1, true);
//     traj_obstacle_colored_pubs.push_back(tmp);

//     ros::Publisher tmp2 = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("/poly_jps_int_" + std::to_string(i), 1, true);
//     jps_poly_pubs.push_back(tmp2);

//     // ros::Publisher tmp3 =
//     //     nh.advertise<visualization_msgs::MarkerArray>("/best_trajectory_found_int_" + std::to_string(i), 1, true);
//     // best_trajectory_found_intervals_pubs.push_back(tmp3);
//   }

//   for (int i = 0; i < num_pol; i++)
//   {
//     ConvexHullsOfCurve tmp2;
//     ConvexHullsOfCurves tmp;

//     tmp2.push_back(hulls_curve[i]);
//     tmp.push_back(tmp2);

//     // convert the obstacles polyhedron arrays
//     decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(cu::vectorGCALPol2vectorJPSPol(tmp));
//     poly_msg.header.frame_id = "odom";
//     jps_poly_pubs[i].publish(poly_msg);
//   }



  // Get the edges of the convex hulls and publish them
//   mt::Edges edges_convex_hulls;



  // publish the trajectories (all the other possible trajectories)
  // trajectories_found_pub.publish(marker_array_all_trajs);

  // ros::spinOnce();

  // ros::spin();

  ros::Rate lr(1000);
  while (ros::ok())
  {
    global_planner.process();
    // global_planner.plan();
    ros::spinOnce();
    lr.sleep();
  }

  return 0;
}