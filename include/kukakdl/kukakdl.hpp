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

#include <kdl/treejnttojacsolver.hpp>
#include <iostream>

#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include <Eigen/Dense>
#include "kukakdl/kukakdl_const.hpp"

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

        KDL::JntArray qd;

        KDL::JntSpaceInertiaMatrix massMatrix;
        KDL::JntArray corioCentriGravTorque;
        KDL::JntArray frictionTorque;
        KDL::JntArray gravityTorque;

        void setJointVelocity(std::vector<double> &qd_des);
        void setLastLinkWithToolInertia(KDL::RotationalInertia toolRInertia,KDL::Vector coordCOG,double m);
        void setExternalToolWrench(KDL::Wrench W_ext);
        void computeMassMatrix();
        void computeCorioCentriGravTorque();
        void computeGravityTorque();
        void computeFrictionTorque();
        
        //  For testing purpose (to compare the dynamics models of the KUKA obtained from SYMORO+ and KDL)
        KDL::ChainDynParam* dynModelSolver;
        KDL::JntSpaceInertiaMatrix massMatrixFromKDL;
        KDL::JntArray corioCentriTorqueFromKDL;
        KDL::JntArray gravityTorqueFromKDL;
        void computeMassMatrixFromKDL();
		void computeCorioCentriTorqueFromKDL();
		void computeGravityTorqueFromKDL();

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

	
	private:
		// Last Link With Tool Inertia parameters
		double XX7_tool;
		double XY7_tool;
		double XZ7_tool;
		double YY7_tool;
		double YZ7_tool;
		double ZZ7_tool; 
		
		// Last Link Wrench (expressed in the last link frame)
		double FX7;
		double FY7;
		double FZ7;
		double CX7;
		double CY7;
		double CZ7; 
		
};

#endif


