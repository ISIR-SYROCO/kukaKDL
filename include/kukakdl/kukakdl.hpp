// Filename:  kukaKDL.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description: 
//

#ifndef KUKAKDL_HPP
#define KUKAKDL_HPP

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <iostream>
#include <Eigen/Dense>

class KukaKDL{
    public:
        KukaKDL();
        KDL::JntArray q;
        KDL::TreeFkSolverPos_recursive* fksolver;
        KDL::TreeJntToJacSolver* treejacsolver;

        KDL::Frame getSegmentPosition(int segment);
        KDL::Frame getSegmentPosition(std::string& segment_name);

        KDL::Jacobian getSegmentJacobian(int segment);
        KDL::Jacobian getSegmentJacobian(std::string& segment_name);

        void setJointPosition(std::vector<double> &q_des);

    private:
        KDL::Tree tree;
        KDL::Chain chain;
        std::vector<std::string> joint_map;
        
};

#endif


