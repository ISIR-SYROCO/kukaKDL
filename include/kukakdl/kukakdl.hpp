// Filename:  kukaKDL.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description: 
//

#ifndef KUKAKDL_HPP
#define KUKAKDL_HPP

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <iostream>
#include <Eigen/Dense>

class KukaKDL{
    public:
        KukaKDL();
        KDL::Tree tree;
        KDL::Jacobian jacobian;
        KDL::JntArray q;
        KDL::ChainFkSolverPos_recursive* fksolver;
        KDL::ChainJntToJacSolver* jacsolver;
        KDL::TreeJntToJacSolver* treejacsolver;

        void computeJacobian();
        
        KDL::Frame getSegmentPosition(int segment);
        KDL::Frame getSegmentPosition(std::string& segment_name);

        KDL::Jacobian getSegmentJacobian(int segment);
        KDL::Jacobian getSegmentJacobian(std::string& segment_name);

        void setJointPosition(std::vector<double> &q_des);


    private:
        KDL::Chain chain;
        std::vector<std::string> joint_map;
        
};

#endif


