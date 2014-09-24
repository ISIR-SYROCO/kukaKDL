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

#include <kdl/chainfksolvervel_recursive.hpp>

#include <Eigen/Dense>
#include "kukakdl/kukakdl_const.hpp"

class KukaKDL{
	
    public:
        KukaKDL();

        /**
         * @brief The kinematic tree of the robot.
         */
        KDL::Tree tree;

        /**
         * @brief The joint space mass matrix for the current joint position.
         */
        KDL::JntSpaceInertiaMatrix massMatrix;

        /**
         * @brief The Coriolis, centrifugal and gravity induced joint torque
         * for the current joint position and velocity.
         */        
        KDL::JntArray corioCentriTorque;

        /**
         * @brief The friction induced joint torque
         * for the current joint velocity.
         */                
        KDL::JntArray frictionTorque;

        /**
         * @brief The gravity induced joint torque
         * for the current joint position.
         */          
        KDL::JntArray gravityTorque;
        
        /**
         * @brief The external wrench induced joint torque
         * for the current joint position.
         */          
        KDL::JntArray externalWrenchTorque;

        int nbSegments();

        KDL::Segment getSegment(int segment);

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

        KDL::Twist getSegmentVelocity(int segment);

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
        KDL::Jacobian getJointJacobian(int segment);

        const std::string& getSegmentName(int index);
        int getSegmentIndex(const std::string name);

        Eigen::VectorXd& getActuatedDofs();
        Eigen::VectorXd& getJointLowerLimits();
        Eigen::VectorXd& getJointUpperLimits();
        KDL::JntArray& getJointPositions();
        KDL::JntArray& getJointVelocities();
        KDL::JntSpaceInertiaMatrix& getInertiaMatrix();
        KDL::JntArray& getNonLinearTerms();
        KDL::JntArray& getGravityTerms();

        /**
         * @brief Set the joint position of the model.
         *
         * @param q_des the joint position in rad.
         */
        void setJointPosition(std::vector<double> &q_des);

        void setLastLinkWithToolInertia(KDL::RotationalInertia toolRInertia,KDL::Vector coordCOG,double m);
        void setExternalToolWrench(KDL::Wrench W_ext);

        /**
         * @brief Set the joint velocity of the model.
         *
         * @param qd_des the joint velocity in rad.
         */
        void setJointVelocity(std::vector<double> &qd_des);
        
        /**
         * @brief Set the external measured wrench.
         *
         * @param f the external measured force.
         * @param t the external measured torque.
         */
        void setExternalMeasuredWrench(std::vector<double> &f, std::vector<double> &t);
        
        /**
         * @brief Set the external point of measure of the wrench wrt to the last link.
         *
         * @param p the  coordinates of the point.
         */
        void setExternalWrenchPoint(std::vector<double> &p);
              
        /**
         * @brief Compute the mass matrix of the model.
         *
         */        
        void computeMassMatrix();
        
        /**
         * @brief Compute the Coriolis, centrifugal and gravity induced joint torque of the model.
         *
         */   
        void computeCorioCentriTorque();
        
        /**
         * @brief Compute the gravity induced joint torque of the model.
         *
         */
        void computeGravityTorque();
        
        /**
         * @brief Compute the friction induced joint torque of the model.
         *
         */
        void computeFrictionTorque();
        
        /**
         * @brief Compute the external wrench induced joint torque of the model.
         *
         */
        void computeExternalWrenchTorque();   

        /**
         * @brief Check if inertia matrix is outdated.
         *
         * @return True if inertia matrix needs to be re computed.
         */
        bool inertiaMatrixOutdated();
        bool corioCentriTorqueOutdated();
        bool gravityOutdated();

    private:
        /**
         * @brief Mapping between segment index and segment name.
         */
        std::vector<std::string> segment_map;
        
        /**
         * @brief The current joint position.
         */
        KDL::JntArray q;
        
         /**
         * @brief The current joint velocity.
         */
        KDL::JntArray qd;
        
        /**
         * @brief The current external measured wrench.
         */
        KDL::Wrench W_ext;
        
        /**
         * @brief The point of application wrt to the last link 
         * of the current external measured wrench.
         */
        KDL::Vector W_ext_point;

        /**
         * @brief The forward kinematic solver.
         */
        KDL::TreeFkSolverPos_recursive* fksolver;
        KDL::ChainFkSolverVel_recursive* fksolvervel;

        /**
         * @brief The jacobian solver.
         */
        KDL::TreeJntToJacSolver* treejacsolver;
        
        /**
         * @brief The dynamic solver.
         */
        KDL::ChainDynParam* dynModelSolver;

        Eigen::VectorXd actuatedDofs;
        Eigen::VectorXd lowerLimits;
        Eigen::VectorXd upperLimits;

        bool inertiaMatrixOutdated_;
        bool corioCentriTorqueOutdated_;
        bool gravityOutdated_;

        void outdate();
};

#endif
