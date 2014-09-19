// Filename:  testkukakdl.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukakdl/kukakdl.hpp"
#include <iostream>
#include <kdl/frames_io.hpp>
#include <Eigen/Eigenvalues> 
#include <Eigen/Dense>

int main(){
    KukaKDL model;
    int segment = 8;
    double q_1[] = {0.0, -0.0523626, 0.0, 1.51845, 0.0, -0.959863, 0.0};
    double q_2[] = {0.829583, 0.118625, 0.0, 1.21193, -1.71438, -0.740072, 1.32882};
    //double q_2[] = {0.,0.,0.,0.,0.,0.,0.};
    std::vector<double> q1(q_1, q_1+7);
    std::vector<double> q2(q_2, q_2+7);
    model.setJointPosition(q1);
    model.computeJacobian();
    model.jacobian.changeBase(model.getSegmentPosition(segment).M.Inverse());

    std::cout << model.getSegmentPosition(segment) << std::endl << std::endl;

    std::cout << model.jacobian.data << std::endl <<std::endl;
    
    model.setJointPosition(q2);
    model.setJointVelocity(q2);
    model.computeJacobian();
    model.jacobian.changeBase(model.getSegmentPosition(segment).M.Inverse()); 

    std::cout << model.getSegmentPosition(segment) << std::endl << std::endl;

    std::cout << model.jacobian.data << std::endl << std::endl;

    std::cout << "Change ref point " << std::endl;
    KDL::Vector v(0.0, -0.3, 0.2);
    model.jacobian.changeRefPoint(v); 
    std::cout << model.jacobian.data << std::endl << std::endl;

	std::cout << "Mass Matrix from SYMORO+ " << std::endl << std::endl;
	model.computeMassMatrix();
	std::cout << model.massMatrix.data << std::endl << std::endl;
	std::cout << "Mass Matrix from SYMORO+ : symmetry test " << std::endl << std::endl;
	std::cout << model.massMatrix.data - model.massMatrix.data.transpose() << std::endl << std::endl;
	std::cout << "Mass Matrix from SYMORO+ : >0 test "<< std::endl << std::endl;
	Eigen::EigenSolver<Eigen::MatrixXd> es(model.massMatrix.data);
	std::cout << "The eigenvalues of the matrix are:" << std::endl << es.eigenvalues() << std::endl << std::endl;
    
    std::cout << "Mass Matrix from SYMORO+ vs from KDL " << std::endl << std::endl;
    std::cout << model.massMatrix.data << std::endl << std::endl;
    model.computeMassMatrixFromKDL();
    std::cout << model.massMatrixFromKDL.data << std::endl << std::endl;
    
    std::cout << "Gravity vector from SYMORO+ vs from KDL " << std::endl << std::endl;
    model.computeGravityTorque();
    std::cout << model.gravityTorque.data << std::endl << std::endl;
    model.computeGravityTorqueFromKDL();
    std::cout << model.gravityTorqueFromKDL.data << std::endl << std::endl;
    
    std::cout << "CoriofugeFric vector from SYMORO+ vs from KDL " << std::endl << std::endl;
    model.computeCorioCentriGravTorque();
    std::cout << model.corioCentriGravTorque.data - model.gravityTorque.data << std::endl << std::endl;
    model.computeCorioCentriTorqueFromKDL();
    std::cout << model.corioCentriTorqueFromKDL.data << std::endl << std::endl;
    
    std::cout << "Fric vector from SYMORO+ " << std::endl << std::endl;
    model.computeFrictionTorque();
    std::cout << model.frictionTorque.data << std::endl << std::endl;
    
    double q_3[] = {0.661933, 0.81477, -0.831109, 1.19049, 0.697476, -1.09382, 0.0};
    double q_4[] = {0., -0.0523486, 0., 1.51845, 0., -0.959969, 0.};
    std::vector<double> q3(q_3, q_3+7);
    std::vector<double> q4(q_4, q_4+7);
    
    model.setJointPosition(q3);
    std::cout << " q: \t " << model.q.data.transpose() << std::endl << std::endl;
    std::cout << "Mass Matrix from SYMORO+ " << std::endl << std::endl;
	model.computeMassMatrix();
	std::cout << model.massMatrix.data << std::endl << std::endl;
	std::cout << "Mass Matrix from KDL " << std::endl << std::endl;
	model.computeMassMatrixFromKDL();
    std::cout << model.massMatrixFromKDL.data << std::endl << std::endl;
	
	model.setJointPosition(q4);
	std::cout << " q: \t" << model.q.data.transpose() << std::endl << std::endl;
    std::cout << "Mass Matrix from SYMORO+ " << std::endl << std::endl;
	model.computeMassMatrix();
	std::cout << model.massMatrix.data << std::endl << std::endl;
	std::cout << "Mass Matrix from KDL " << std::endl << std::endl;
	model.computeMassMatrixFromKDL();
    std::cout << model.massMatrixFromKDL.data << std::endl << std::endl;
    
    return 0;

}
