// Filename:  testkukakdl.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukakdl/kukakdl.hpp"
#include <iostream>
#include <kdl/frames_io.hpp>
#include <Eigen/Eigenvalues> 
#include <Eigen/Dense>

void computeSegmentJacobian(KukaKDL& model){
    for(int i=0; i<8; ++i){
        std::cout << "Segment Jacobian " << i << std::endl;
        KDL::Jacobian j = model.getSegmentJacobian(i);
        j.changeBase(model.getSegmentPosition(i).M.Inverse()); 
        std::cout << j.data << std::endl;
    }
}

int main(){
    KukaKDL model;
    int segment = 8;
    double q_1[] = {0.0, -0.0523626, 0.0, 1.51845, 0.0, -0.959863, 0.0};
    //double q_2[] = {0.829583, 0.118625, 0.0, 1.21193, -1.71438, -0.740072, 1.32882};
    double q_2[] = {0.,0.,0.,0.,0.,0.,0.};

    std::vector<double> q1(q_1, q_1+7);
    std::vector<double> q2(q_2, q_2+7);
    model.setJointPosition(q2);
    computeSegmentJacobian(model);

    //model.setJointPosition(q1);

    //computeSegmentJacobian(model);

    model.setJointVelocity(q2);

	std::cout << "Mass Matrix " << std::endl << std::endl;
	model.computeMassMatrix();
	std::cout << model.massMatrix.data << std::endl << std::endl;
	std::cout << "Mass Matrix : symmetry test " << std::endl << std::endl;
	std::cout << model.massMatrix.data - model.massMatrix.data.transpose() << std::endl << std::endl;
	std::cout << "Mass Matrix : >0 test "<< std::endl << std::endl;
	Eigen::EigenSolver<Eigen::MatrixXd> es(model.massMatrix.data);
	std::cout << "The eigenvalues of the matrix are:" << std::endl << es.eigenvalues() << std::endl << std::endl;
    
    std::cout << "Gravity vector " << std::endl << std::endl;
    model.computeGravityTorque();
    std::cout << model.gravityTorque.data << std::endl << std::endl;

    std::cout << "Coriofuge vector  " << std::endl << std::endl;
    model.computeCorioCentriTorque();
    std::cout << model.corioCentriTorque.data << std::endl << std::endl;
    
    std::cout << "Fric vector " << std::endl << std::endl;
    model.computeFrictionTorque();
    std::cout << model.frictionTorque.data << std::endl << std::endl;
    
    double f_[] = {10.0,0.,0.};
    double t_[] = {0.,0.,0.};
    double p_[] = {0.,0.,0.1};
    std::vector<double> f(f_,f_+3);
    std::vector<double> t(t_,t_+3);
    std::vector<double> p(p_,p_+3);
    
    model.setExternalWrenchPoint(p);
    model.setExternalMeasuredWrench(f,t);
    
    std::cout << "External Wrench Torque " << std::endl << std::endl;
    model.computeExternalWrenchTorque();
    std::cout << model.externalWrenchTorque.data << std::endl << std::endl;
    
    return 0;

}
