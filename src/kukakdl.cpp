// Filename:  kukaKDL.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukakdl.hpp"

KukaKDL::KukaKDL(){
    chain.addSegment(KDL::Segment("world",KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Rotation::RPY( 0,0,3.14159265359/6),KDL::Vector(0,  0, 0)))); 
    chain.addSegment(KDL::Segment("world_robot",KDL::Joint("shoulder_0",KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY( 0,0,3.14159265359),KDL::Vector(0,  0, 0.3105)))); 
    chain.addSegment(KDL::Segment("segment_1",KDL::Joint("shoulder_1",KDL::Joint::RotY),KDL::Frame(KDL::Rotation::RPY( 0,0,0),KDL::Vector(0,  0,  0))));
    chain.addSegment(KDL::Segment("segment_2",KDL::Joint("shoulder_2",KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(  0,0,3.14159265359),KDL::Vector(0, 0, 0.4))));
    chain.addSegment(KDL::Segment("segment_3",KDL::Joint("elbow_0",KDL::Joint::RotY),KDL::Frame(KDL::Rotation::RPY(0, 0, 0),KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment("segment_4",KDL::Joint("wrist_0",KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0,  0, -1.570796326795),KDL::Vector(0, 0, 0.39))));
    chain.addSegment(KDL::Segment("segment_5",KDL::Joint("wrist_1",KDL::Joint::RotX),KDL::Frame(KDL::Rotation::RPY(0, 0, 0),KDL::Vector(0,  0,  0))));
    chain.addSegment(KDL::Segment("segment_6",KDL::Joint("wrist_2",KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0,  0,  0),KDL::Vector(0.016617,  -0.016932,  0.238283))));
    chain.addSegment(KDL::Segment("segment_7",KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Rotation::RPY( 0,0,0),KDL::Vector(0,  0,  0))));

    fksolver = new KDL::ChainFkSolverPos_recursive(chain);
    jacsolver = new KDL::ChainJntToJacSolver(chain);
    q.resize(chain.getNrOfJoints());
    jacobian.resize(chain.getNrOfJoints());
}

void KukaKDL::setJointPosition(std::vector<double> &q_des){
    for(unsigned int i=0; i<chain.getNrOfJoints(); i++){
        q(i) = q_des[i];
    }
}

void KukaKDL::computeJacobian(){
    jacsolver->JntToJac(q, jacobian);
}

KDL::Frame KukaKDL::getSegmentPosition(int segment){
    KDL::Frame cart_pos;
    fksolver->JntToCart(q, cart_pos, segment);
    return cart_pos;
}
