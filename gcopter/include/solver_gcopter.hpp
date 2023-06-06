#ifndef SOLVER_GCOPTER_HPP
#define SOLVER_GCOPTER_HPP
#include <Eigen/Dense>

#include <iomanip>  //set precision
#include "rmader_types.hpp"
#include "utils.hpp"
#include "timer.hpp"
#include <decomp_geometry/polyhedron.h>  //For Polyhedron  and Hyperplane definition
#include "separator.hpp"
#include "octopus_search.hpp"
#include "solver_params.hpp"

typedef RMADER_timers::Timer MyTimer;


class SolverGCopter
{

public:

    SolverGCopter(ms::par_solver &par);

    ~SolverGCopter();

    bool optimize(bool &is_stuck, bool &is_A_star_failed, bool &is_q0_fail);

    void setHulls(mt::ConvexHullsOfCurve_Std &hulls);

    mt::trajectory traj_solution_;
    
}
