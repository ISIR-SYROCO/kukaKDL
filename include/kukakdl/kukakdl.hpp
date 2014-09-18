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
        
        /**
         * @brief Return a segment position of the robot.
         *
         * @param segment the index of the segment.
         */
        KDL::Frame getSegmentPosition(int segment);

        /**
         * @brief Return a segment position of the robot.
         *
         * @param segment_name the name of the segment.
         */
        KDL::Frame getSegmentPosition(std::string& segment_name);

        /**
         * @brief Return the jacobian expressed in the base frame 
         *        with the reference point at the end of the segment.
         *
         * @param segment the index of the segment.
         */
        KDL::Jacobian getSegmentJacobian(int segment);

        /**
         * @brief Return the jacobian expressed in the base frame 
         *        with the reference point at the end of the segment.
         *
         * @param segment_name the name of the segment.
         */
        KDL::Jacobian getSegmentJacobian(std::string& segment_name);

        /**
         * @brief Set the joint position of the model.
         *
         * @param q_des the joint position in rad.
         */
        void setJointPosition(std::vector<double> &q_des);

    private:
        /**
         * @brief The kinematic tree of the robot.
         */
        KDL::Tree tree;

        /**
         * @brief Mapping between segment index and segment name.
         */
        std::vector<std::string> segment_map;
        
        /**
         * @brief The current joint position.
         */
        KDL::JntArray q;

        /**
         * @brief The forward kinematic solver.
         */
        KDL::TreeFkSolverPos_recursive* fksolver;

        /**
         * @brief The jacobian solver.
         */
        KDL::TreeJntToJacSolver* treejacsolver;

};

#endif


