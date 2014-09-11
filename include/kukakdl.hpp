// Filename:  kukaKDL.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description: 
//

#ifndef KUKAKDL_HPP
#define KUKAKDL_HPP

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Dense>

class KukaKDL{
    public:
        KukaKDL();
        KDL::Chain chain;
        KDL::Jacobian jacobian;
        KDL::JntArray q;
        KDL::ChainFkSolverPos_recursive* fksolver;
        KDL::ChainJntToJacSolver* jacsolver;

        void computeJacobian();
        void setJointPosition(std::vector<double> &q_des);

        KDL::Frame getSegmentPosition(int segment);
};

#endif


