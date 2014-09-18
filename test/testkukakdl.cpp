// Filename:  testkukakdl.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukakdl/kukakdl.hpp"
#include <iostream>
#include <kdl/frames_io.hpp>

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
    double q_2[] = {0.829583, -0.118625, 0.1, 1.21193, -1.71438, -0.740072, 1.32882};
    std::vector<double> q1(q_1, q_1+7);
    std::vector<double> q2(q_2, q_2+7);
    model.setJointPosition(q1);
    computeSegmentJacobian(model);

    model.setJointPosition(q2);
    computeSegmentJacobian(model);
    return 0;

}
