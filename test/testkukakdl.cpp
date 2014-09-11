// Filename:  testkukakdl.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukakdl.hpp"
#include <iostream>

int main(){
    KukaKDL model;
    std::vector<double> q(7);
    for(int i=0; i<7;++i){
        q[i] = i*0.2;
    }
    model.setJointPosition(q);
    model.computeJacobian();
    std::cout << model.jacobian.data << std::endl;
    return 0;
}
