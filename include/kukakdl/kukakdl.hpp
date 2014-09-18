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
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <Eigen/Dense>
#include "kukakdl/kukakdl_const.hpp"

class KukaKDL{
	
    public:
        KukaKDL();
        KDL::Chain chain;
        KDL::Jacobian jacobian;
        KDL::JntArray q;
        KDL::JntArray qd;
        KDL::ChainFkSolverPos_recursive* fksolver;
        KDL::ChainJntToJacSolver* jacsolver;
        KDL::JntSpaceInertiaMatrix massMatrix;
        KDL::JntArray corioCentriGravTorque;
        KDL::JntArray frictionTorque;
        KDL::JntArray gravityTorque;

        void computeJacobian();
        void setJointPosition(std::vector<double> &q_des);
        void setJointVelocity(std::vector<double> &qd_des);
        void setLastLinkWithToolInertia(KDL::RotationalInertia toolRInertia,KDL::Vector coordCOG,double m);
        void setExternalToolWrench(KDL::Wrench W_ext);
        void computeMassMatrix();
        void computeCorioCentriGravTorque();
        void computeGravityTorque();
        void computeFrictionTorque();
        KDL::Frame getSegmentPosition(int segment);
        
        //  For testing purpose (to compare the dynamics models of the KUKA obtained from SYMORO+ and KDL)
        KDL::ChainDynParam* dynModelSolver;
        KDL::JntSpaceInertiaMatrix massMatrixFromKDL;
        KDL::JntArray corioCentriTorqueFromKDL;
        KDL::JntArray gravityTorqueFromKDL;
        void computeMassMatrixFromKDL();
		void computeCorioCentriTorqueFromKDL();
		void computeGravityTorqueFromKDL();
	
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


