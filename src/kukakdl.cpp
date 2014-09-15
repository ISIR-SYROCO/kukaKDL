// Filename:  kukaKDL.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukakdl/kukakdl.hpp"

KukaKDL::KukaKDL(){
    //joint 0
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),
                KDL::Frame::DH_Craig1989(0.0, 0.0, 0.31, 0.0)
                ));
    //joint 1
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector::Zero(),
                    KDL::RotationalInertia(0.0,0.0,0.0115343,0.0,0.0,0.0))));

    //joint 2 
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector(0.0,-0.3120511,-0.0038871),
                    KDL::RotationalInertia(-0.5471572,-0.0000302,-0.5423253,0.0,0.0,0.0018828))));

    //joint 3
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector(0.0,-0.0015515,0.0),
                    KDL::RotationalInertia(0.0063507,0.0,0.0107804,0.0,0.0,-0.0005147))));

    //joint 4
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector(0.0,0.5216809,0.0),
                    KDL::RotationalInertia(-1.0436952,0.0,-1.0392780,0.0,0.0,0.0005324))));

    //joint 5
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector(0.0,0.0119891,0.0),
                    KDL::RotationalInertia(0.0036654,0.0,0.0060429,0.0,0.0,0.0004226))));

    //joint 6
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector(0.0,0.0080787,0.0),
                    KDL::RotationalInertia(0.0010431,0.0,0.0036376,0.0,0.0,0.0000101))));
    //joint 7
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, 0.0, 0.078, 0.0),
                KDL::Frame::DH_Craig1989(0.0, 0.0, 0.078, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector::Zero(),
                    KDL::RotationalInertia(0.000001,0.0,0.0001203,0.0,0.0,0.0))));

    //tool
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::Identity()));

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
