// Filename:  orckukakdl.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukakdl/orckukakdl.hpp"
#include "kukakdl/kukakdl.hpp"

#include <kdl/joint.hpp>
#include <Eigen/Lgsm>

Eigen::Displacementd KDLFrameToDisplacement(KDL::Frame frame){
    double qw, qx, qy, qz;
    frame.M.GetQuaternion(qx, qy, qz, qw);
    return Eigen::Displacementd(frame.p.x(), frame.p.y(), frame.p.z(), qw, qx, qy, qz);
}

Eigen::Twistd KDLTwistToTwist(KDL::Twist twist){
    return Eigen::Twistd(twist.rot.x(), twist.rot.y(), twist.rot.z(),
                         twist.vel.x(), twist.vel.y(), twist.vel.z());
}

int OrcKukaKDL::nbSegments(){
    return kdlmodel.nbSegments();
}

Eigen::VectorXd& OrcKukaKDL::getActuatedDofs(){
    return kdlmodel.getActuatedDofs();
}

Eigen::VectorXd& OrcKukaKDL::getJointLowerLimits(){
    return kdlmodel.getJointLowerLimits();
}

Eigen::VectorXd& OrcKukaKDL::getJointUpperLimits(){
    return kdlmodel.getJointUpperLimits();
}

Eigen::VectorXd& OrcKukaKDL::getJointPositions(){
    return kdlmodel.getJointPositions().data;
}

Eigen::VectorXd& OrcKukaKDL::getJointVelocities(){
    return kdlmodel.getJointVelocities().data;
}

Eigen::Displacementd& OrcKukaKDL::getFreeFlyerPosition(){
    return H_root;
}

Eigen::Twistd& OrcKukaKDL::getFreeFlyerVelocity(){
    return T_root;
}

Eigen::MatrixXd& OrcKukaKDL::getInertiaMatrix(){
    return kdlmodel.getInertiaMatrix().data;
}

Eigen::MatrixXd OrcKukaKDL::getInertiaMatrixInverse(){
    return kdlmodel.getInertiaMatrix().data.inverse();
}

const Eigen::MatrixXd& OrcKukaKDL::getDampingMatrix() const{
    return B;
}

Eigen::VectorXd& OrcKukaKDL::getNonLinearTerms(){
    return kdlmodel.getNonLinearTerms().data;
}

const Eigen::VectorXd& OrcKukaKDL::getLinearTerms() const{
    return l;
}

Eigen::VectorXd& OrcKukaKDL::getGravityTerms(){
    return kdlmodel.getGravityTerms().data;
}

double OrcKukaKDL::getSegmentMass(int index){
    return kdlmodel.getSegment(index).getInertia().getMass();
}

Eigen::Displacementd OrcKukaKDL::getSegmentPosition(int index){
    return KDLFrameToDisplacement(kdlmodel.getSegmentPosition(index));
}

Eigen::Twistd OrcKukaKDL::getSegmentVelocity(int index){
    return KDLTwistToTwist(kdlmodel.getSegmentVelocity(index));
}

Eigen::Matrix<double,6,Eigen::Dynamic> OrcKukaKDL::getSegmentJacobian(int index){
    KDL::Jacobian kdljac = kdlmodel.getSegmentJacobian(index);
    Eigen::Matrix<double, 6, 7> jac;
    jac.block<3, 7>(0, 0) =  kdljac.data.block<3, 7>(3, 0);
    jac.block<3, 7>(3, 0) =  kdljac.data.block<3, 7>(0, 0);
    return jac;
}

Eigen::Matrix<double,6,Eigen::Dynamic>  OrcKukaKDL::getJointJacobian(int index){
    KDL::Jacobian kdljac = kdlmodel.getJointJacobian(index);
    Eigen::Matrix<double, 6, 7> jac;
    jac.block<3, 7>(0, 0) =  kdljac.data.block<3, 7>(3, 0);
    jac.block<3, 7>(3, 0) =  kdljac.data.block<3, 7>(0, 0);
    return jac;
}
