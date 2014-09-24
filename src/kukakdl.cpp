// Filename:  kukaKDL.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukakdl/kukakdl.hpp"

KukaKDL::KukaKDL(){
    segment_map.push_back("base");
    segment_map.push_back("00");
    segment_map.push_back("01");
    segment_map.push_back("02");
    segment_map.push_back("03");
    segment_map.push_back("04");
    segment_map.push_back("05");
    segment_map.push_back("06");
    segment_map.push_back("07");

    KDL::Chain chain;

    //joint 0
    chain.addSegment(KDL::Segment("base", KDL::Joint(KDL::Joint::None),
                KDL::Frame::DH_Craig1989(0.0, ALPHA0, R0, 0.0)
                ));

    //joint 1
	chain.addSegment(KDL::Segment("00", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, ALPHA1, R1, 0.0),
                KDL::Frame::DH_Craig1989(0.0, ALPHA1, R1, 0.0).Inverse()*KDL::RigidBodyInertia(M1_KDL,
                    KDL::Vector(COGX1_KDL, COGY1_KDL, COGZ1_KDL),
                    KDL::RotationalInertia(XX1_KDL,YY1_KDL,ZZ1_KDL,XY1_KDL,XZ1_KDL,YZ1_KDL))));

    //joint 2 
    chain.addSegment(KDL::Segment("01", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0,  ALPHA2, R2, 0.0),
                KDL::Frame::DH_Craig1989(0.0,  ALPHA2, R2, 0.0).Inverse()*KDL::RigidBodyInertia(M2_KDL,
                    KDL::Vector(COGX2_KDL, COGY2_KDL, COGZ2_KDL),
                    KDL::RotationalInertia(XX2_KDL,YY2_KDL,ZZ2_KDL,XY2_KDL,XZ2_KDL,YZ2_KDL))));

    //joint 3
    chain.addSegment(KDL::Segment("02", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0,  ALPHA3, R3, 0.0),
                KDL::Frame::DH_Craig1989(0.0,  ALPHA3, R3, 0.0).Inverse()*KDL::RigidBodyInertia(M3_KDL,
                    KDL::Vector(COGX1_KDL, COGY1_KDL, COGZ1_KDL),
                    KDL::RotationalInertia(XX1_KDL,YY1_KDL,ZZ1_KDL,XY1_KDL,XZ1_KDL,YZ1_KDL))));

    //joint 4
    chain.addSegment(KDL::Segment("03", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, ALPHA4, R4, 0.0),
                KDL::Frame::DH_Craig1989(0.0, ALPHA4, R4, 0.0).Inverse()*KDL::RigidBodyInertia(M4_KDL,
                    KDL::Vector(COGX4_KDL, COGY4_KDL, COGZ4_KDL),
                    KDL::RotationalInertia(XX4_KDL,YY4_KDL,ZZ4_KDL,XY4_KDL,XZ4_KDL,YZ4_KDL))));

    //joint 5
    chain.addSegment(KDL::Segment("04", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0,  ALPHA5, R5, 0.0),
                KDL::Frame::DH_Craig1989(0.0,  ALPHA5, R5, 0.0).Inverse()*KDL::RigidBodyInertia(M5_KDL,
                    KDL::Vector(COGX5_KDL, COGY5_KDL, COGZ5_KDL),
                    KDL::RotationalInertia(XX5_KDL,YY5_KDL,ZZ5_KDL,XY5_KDL,XZ5_KDL,YZ5_KDL))));

    //joint 6
    chain.addSegment(KDL::Segment("05", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, ALPHA6, R6, 0.0),
                KDL::Frame::DH_Craig1989(0.0, ALPHA6, R6, 0.0).Inverse()*KDL::RigidBodyInertia(M6_KDL,
                    KDL::Vector(COGX6_KDL, COGY6_KDL, COGZ6_KDL),
                    KDL::RotationalInertia(XX6_KDL,YY6_KDL,ZZ6_KDL,XY6_KDL,XZ6_KDL,YZ6_KDL))));
    //joint 7
    chain.addSegment(KDL::Segment("06", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, ALPHA7, R7, 0.0),
                KDL::Frame::DH_Craig1989(0.0, ALPHA7, R7, 0.0).Inverse()*KDL::RigidBodyInertia(M7_KDL,
                    KDL::Vector(COGX7_KDL, COGY7_KDL, COGZ7_KDL),
                    KDL::RotationalInertia(XX7_KDL,YY7_KDL,ZZ7_KDL,XY7_KDL,XZ7_KDL,YZ7_KDL))));

    //tool
    chain.addSegment(KDL::Segment("07", KDL::Joint(KDL::Joint::None), KDL::Frame::Identity()));
    

    tree.addChain(chain, "root");

    treejacsolver = new KDL::TreeJntToJacSolver(tree);
    fksolver = new KDL::TreeFkSolverPos_recursive(tree);
    fksolvervel = new KDL::ChainFkSolverVel_recursive(chain);

    q.resize(chain.getNrOfJoints());


	dynModelSolver = new KDL::ChainDynParam(chain,KDL::Vector(0.,0.,-9.81));
    qd.resize(chain.getNrOfJoints());
    externalWrenchTorque.resize(chain.getNrOfJoints());
    massMatrix.resize(chain.getNrOfJoints());
    corioCentriTorque.resize(chain.getNrOfJoints());
    gravityTorque.resize(chain.getNrOfJoints());
    frictionTorque.resize(chain.getNrOfJoints());

    actuatedDofs = Eigen::VectorXd::Constant(tree.getNrOfJoints(), 1);
    lowerLimits.resize(tree.getNrOfJoints());
    lowerLimits << -2.97, -2.10, -2.97, -2.10, -2.97, -2.10, -2.97;
    upperLimits.resize(tree.getNrOfJoints());
    upperLimits << 2.97, 2.10, 2.97, 2.10, 2.97, 2.10, 2.97;


    outdate();
}

int KukaKDL::nbSegments(){
    return tree.getNrOfSegments();
}

KDL::Segment KukaKDL::getSegment(int segment){
    return tree.getSegment(segment_map[segment])->second.segment;
}

KDL::Frame KukaKDL::getSegmentPosition(int segment){
    KDL::Frame cart_pos;
    fksolver->JntToCart(q, cart_pos, segment_map[segment]);
    return cart_pos;
}

KDL::Frame KukaKDL::getSegmentPosition(std::string& segment_name){
    KDL::Frame cart_pos;
    fksolver->JntToCart(q, cart_pos, segment_name);
    return cart_pos;
}

KDL::Twist KukaKDL::getSegmentVelocity(int segment){
    KDL::FrameVel cart_vel;
    KDL::JntArrayVel vel(q, qd);
    fksolvervel->JntToCart(vel, cart_vel, segment);
    return cart_vel.GetTwist();
}

KDL::Jacobian KukaKDL::getSegmentJacobian(int segment){
    KDL::Jacobian j(tree.getNrOfJoints());
    treejacsolver->JntToJac(q, j, segment_map[segment]);
    return j;
}

KDL::Jacobian KukaKDL::getSegmentJacobian(std::string& segment_name){
    KDL::Jacobian j(tree.getNrOfJoints());
    treejacsolver->JntToJac(q, j, segment_name);
    return j;
}

KDL::Jacobian KukaKDL::getJointJacobian(int segment){
    return getSegmentJacobian(segment);
}

const std::string& KukaKDL::getSegmentName(int index){
    return segment_map[index];
}

int KukaKDL::getSegmentIndex(const std::string name){
    for(int i=0; i<tree.getNrOfJoints(); ++i){
        if(segment_map[i] == name){
            return i;
        }
    }
    return -1;
}

Eigen::VectorXd& KukaKDL::getActuatedDofs(){
    return actuatedDofs;
}

Eigen::VectorXd& KukaKDL::getJointLowerLimits(){
    return lowerLimits;
}

Eigen::VectorXd& KukaKDL::getJointUpperLimits(){
    return upperLimits;
}

KDL::JntArray& KukaKDL::getJointPositions(){
    return q;
}

KDL::JntArray& KukaKDL::getJointVelocities(){
    return qd;
}

KDL::JntSpaceInertiaMatrix& KukaKDL::getInertiaMatrix(){
    if(inertiaMatrixOutdated_ == true){
        computeMassMatrix();
        inertiaMatrixOutdated_ = false;
    }
    return massMatrix;
}

KDL::JntArray& KukaKDL::getNonLinearTerms(){
    if(corioCentriTorqueOutdated_ == true){
        computeCorioCentriTorque();
        corioCentriTorqueOutdated_ = false;
    }
    return corioCentriTorque;
}

KDL::JntArray& KukaKDL::getGravityTerms(){
    if(gravityOutdated_ == true){
        computeGravityTorque();
        gravityOutdated_ = false;
    }
    return gravityTorque;
}

void KukaKDL::setJointPosition(std::vector<double> &q_des){
    outdate();
    for(unsigned int i=0; i<tree.getNrOfJoints(); i++){
        q(i) = q_des[i];
    }
}

void KukaKDL::setJointVelocity(std::vector<double> &qd_des){
    outdate();
    for(unsigned int i=0; i<tree.getNrOfJoints(); i++){
        qd(i) = qd_des[i];
    }
}

void KukaKDL::setExternalMeasuredWrench(std::vector<double> &f, std::vector<double> &t){
	for(unsigned int i=0; i<3; i++){
        W_ext.force(i) = f[i];
        W_ext.torque(i) = t[i];
    }
}

void KukaKDL::setExternalWrenchPoint(std::vector<double> &p){
	for(unsigned int i=0; i<3; i++){
        W_ext_point(i) = p[i];
    }
}

void KukaKDL::computeMassMatrix(){
    dynModelSolver->JntToMass(q,massMatrix);
}

void KukaKDL::computeCorioCentriTorque(){
    dynModelSolver->JntToCoriolis(q,qd,corioCentriTorque);
}

void KukaKDL::computeGravityTorque(){
    dynModelSolver->JntToGravity(q,gravityTorque);
}

void KukaKDL::computeExternalWrenchTorque(){
	
    KDL::Jacobian j(tree.getNrOfJoints());
    treejacsolver->JntToJac(q, j, "06");
    
    KDL::Wrench w = W_ext.RefPoint(-W_ext_point);
	Eigen::Vector3d f,t;
	
    for(unsigned int i=0;i<3;i++){
		f(i) = w.force.data[i];
		t(i) = w.torque.data[i];
	}
   
    externalWrenchTorque.data = j.data.transpose().block(0,0,7,3) * f +
								j.data.transpose().block(0,3,7,3) * t;
}

void KukaKDL::computeFrictionTorque(){
	
	 frictionTorque.data <<  	FV1*qd(0) + FS1*sign(qd(0)),
	 							FV2*qd(1) + FS2*sign(qd(1)),
	 							FV3*qd(2) + FS3*sign(qd(2)),
								FV4*qd(3) + FS4*sign(qd(3)),
								FV5*qd(4) + FS5*sign(qd(4)),
								FV6*qd(5) + FS6*sign(qd(5)),
								FV7*qd(6) + FS7*sign(qd(6));
}

bool KukaKDL::inertiaMatrixOutdated(){
    return inertiaMatrixOutdated_;
}

bool KukaKDL::corioCentriTorqueOutdated(){
    return corioCentriTorqueOutdated_;
}

bool KukaKDL::gravityOutdated(){
    return gravityOutdated_;
}

void KukaKDL::outdate(){
    inertiaMatrixOutdated_ = true;
    corioCentriTorqueOutdated_ = true;
    gravityOutdated_ = true;
}

