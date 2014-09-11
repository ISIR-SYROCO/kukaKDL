// Filename:  testkukakdl.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukakdl.hpp"
#include <iostream>
#include <kdl/frames_io.hpp>

int main(){
    KukaKDL model;
    int segment = 8;
    double q_1[] = {0.0, -0.0523626, 0.0, 1.51845, 0.0, -0.959863, 0.0};
    double q_2[] = {0.829583, 0.118625, 0.0, 1.21193, -1.71438, -0.740072, 1.32882};
    std::vector<double> q1(q_1, q_1+7);
    std::vector<double> q2(q_2, q_2+7);
    model.setJointPosition(q1);
    model.computeJacobian();
    model.jacobian.changeBase(model.getSegmentPosition(segment).M.Inverse());

    std::cout << model.getSegmentPosition(segment) << std::endl << std::endl;

    std::cout << model.jacobian.data << std::endl <<std::endl;
    
    model.setJointPosition(q2);
    model.computeJacobian();
    model.jacobian.changeBase(model.getSegmentPosition(segment).M.Inverse()); 

    std::cout << model.getSegmentPosition(segment) << std::endl << std::endl;

    std::cout << model.jacobian.data << std::endl << std::endl;
    
    return 0;

}
