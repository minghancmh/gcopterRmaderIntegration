#include "solver_gcopter.hpp"
#include "termcolor.hpp"
#include "bspline_utils.hpp"
#include "ros/ros.h"

#include <decomp_util/ellipsoid_decomp.h>  //For Polyhedron definition
#include <unsupported/Eigen/Splines>
#include <iostream>
#include <list>
#include <random>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace termcolor;

SolverGCopter::SolverGCopter(ms::par_solver &par)
{
    par_ = par;
    p_ = par_.deg_pol;
    M_ = par_.num_pol + 2 * p_;
    N_ = M_ - p_ -1;
    num_of_segments_ = (M_ - 2 * p_);

    ///////////////////////////////////////
    mt::basisConverter basis_converter;

    // basis used for collision
    if (par_.basis == "MINVO")
    {
        basis_ = MINVO;
        M_pos_bs2basis_ = basis_converter.getMinvoPosConverters(num_of_segments_);
        M_vel_bs2basis_ = basis_converter.getMinvoVelConverters(num_of_segments_);
    }
    else if (par_.basis == "BEZIER")
    {
        basis_ = BEZIER;
        M_pos_bs2basis_ = basis_converter.getBezierPosConverters(num_of_segments_);
        M_vel_bs2basis_ = basis_converter.getBezierVelConverters(num_of_segments_);
    }
    else if (par_.basis == "B_SPLINE")
    {
        basis_ = B_SPLINE;
        M_pos_bs2basis_ = basis_converter.getBSplinePosConverters(num_of_segments_);
        M_vel_bs2basis_ = basis_converter.getBSplineVelConverters(num_of_segments_);
    }
    else
    {
        std::cout << red << "Basis " << par_.basis << " not implemented yet" << reset << std::endl;
        std::cout << red << "============================================" << reset << std::endl;
        abort();
    }

    A_pos_bs_ = basis_converter.getABSpline(num_of_segments_);

    separator_solver_ = new separator::Separator();
    octopusSolver_ = new OctopusSearch(par_.basis, par_.num_pol, par_.deg_pol, par_.alpha_shrink);

}

SolverGCopter::~SolverGCopter()
{

}