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
                KDL::Frame::DH_Craig1989(0.0, ALPHA1, R1, 0.0)
                ));

    //joint 1
    chain.addSegment(KDL::Segment("00", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector::Zero(),
                    KDL::RotationalInertia(0.0,0.0,0.0115343,0.0,0.0,0.0))));

    //joint 2 
    chain.addSegment(KDL::Segment("01", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector(0.0,-0.3120511,-0.0038871),
                    KDL::RotationalInertia(-0.5471572,-0.0000302,-0.5423253,0.0,0.0,0.0018828))));

    //joint 3
    chain.addSegment(KDL::Segment("02", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector(0.0,-0.0015515,0.0),
                    KDL::RotationalInertia(0.0063507,0.0,0.0107804,0.0,0.0,-0.0005147))));

    //joint 4
    chain.addSegment(KDL::Segment("03", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector(0.0,0.5216809,0.0),
                    KDL::RotationalInertia(-1.0436952,0.0,-1.0392780,0.0,0.0,0.0005324))));

    //joint 5
    chain.addSegment(KDL::Segment("04", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector(0.0,0.0119891,0.0),
                    KDL::RotationalInertia(0.0036654,0.0,0.0060429,0.0,0.0,0.0004226))));

    //joint 6
    chain.addSegment(KDL::Segment("05", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector(0.0,0.0080787,0.0),
                    KDL::RotationalInertia(0.0010431,0.0,0.0036376,0.0,0.0,0.0000101))));
    //joint 7
    chain.addSegment(KDL::Segment("06", KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(0.0, 0.0, 0.078, 0.0),
                KDL::Frame::DH_Craig1989(0.0, 0.0, 0.078, 0.0).Inverse()*KDL::RigidBodyInertia(2,
                    KDL::Vector::Zero(),
                    KDL::RotationalInertia(0.000001,0.0,0.0001203,0.0,0.0,0.0))));

    //tool
    chain.addSegment(KDL::Segment("07", KDL::Joint(KDL::Joint::None), KDL::Frame::Identity()));

    tree.addChain(chain, "root");

    treejacsolver = new KDL::TreeJntToJacSolver(tree);
    fksolver = new KDL::TreeFkSolverPos_recursive(tree);
    fksolvervel = new KDL::ChainFkSolverVel_recursive(chain);

    q.resize(chain.getNrOfJoints());


    qd.resize(chain.getNrOfJoints());
    
    massMatrix.resize(chain.getNrOfJoints());
    corioCentriGravTorque.resize(chain.getNrOfJoints());
    gravityTorque.resize(chain.getNrOfJoints());
    frictionTorque.resize(chain.getNrOfJoints());
    
    // For test purpose
    dynModelSolver = new KDL::ChainDynParam(chain,KDL::Vector(0.,0.,-9.81));
    massMatrixFromKDL.resize(chain.getNrOfJoints());
    corioCentriTorqueFromKDL.resize(chain.getNrOfJoints());
    gravityTorqueFromKDL.resize(chain.getNrOfJoints());
    
    // Last Link With Tool Inertia parameters (set to the value identified without tool by default)
    XX7_tool = XX7;
	XY7_tool = XY7;
	XZ7_tool = XZ7;
	YY7_tool = YY7;
	YZ7_tool = YZ7;
	ZZ7_tool = ZZ7;
	
	// Last Link External Wrench is set to 0. by default
	FX7 = 0.;
	FY7 = 0.;
	FZ7 = 0.;
	CX7 = 0.;
	CY7 = 0.;
	CZ7 = 0.;

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

Eigen::VectorXd& KukaKDL::getActuatedDofs(){
    return actuatedDofs;
}

Eigen::VectorXd& KukaKDL::getJointLowerLimits(){
    return lowerLimits;
}

Eigen::VectorXd& KukaKDL::getJointUpperLimits(){
    return upperLimits;
}

Eigen::VectorXd& KukaKDL::getJointPositions(){
    return q.data;
}

Eigen::VectorXd& KukaKDL::getJointVelocities(){
    return qd.data;
}

KDL::JntSpaceInertiaMatrix& KukaKDL::getInertiaMatrix(){
    if(inertiaMatrixOutdated == true){
        computeMassMatrixFromKDL();
        inertiaMatrixOutdated = false;
    }
    return massMatrixFromKDL;
}

KDL::JntArray& KukaKDL::getNonLinearTerms(){
    if(corioCentriTorqueOutdated == true){
        computeCorioCentriTorqueFromKDL();
        corioCentriTorqueOutdated = false;
    }
    return corioCentriTorqueFromKDL;
}

KDL::JntArray& KukaKDL::getGravityTerms(){
    if(gravityOutdated == true){
        computeGravityTorqueFromKDL();
        gravityOutdated = false;
    }
    return gravityTorqueFromKDL;
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

void KukaKDL::setLastLinkWithToolInertia(KDL::RotationalInertia toolRInertia,KDL::Vector coordCOG,double m){
	// toolRInertia represent the rotational inertia matrix of the tool expressed at the COG
	// coordCOG is the coordinates of the COG of the tool expressed in the last frame
	// m is the mass of the tool
	
	// Link 7 is considered to have pure rotational diagonal inertia (no mass)
	// Adding the tool, the total inertia expressed in the last frame is given by the König-Huygens Theorem
	// I_total/R7 = 
	// I_7/R7 +
	// I_tool/COG +
	// [m_tool *(coordCOG_z^2 + coordCOG_y^2)], -m_tool*(coordCOG_y*coordCOG_x), -m_tool*(coordCOG_z*coordCOG_x);
	//  -m_tool*(coordCOG_x*coordCOG_y), m_tool*(coordCOG_z^2 + coordCOG_x^2)], -m_tool*(coordCOG_z*coordCOG_y);
	//  -m_tool*(coordCOG_x*coordCOG_z), -m_tool*(coordCOG_y*coordCOG_z),  m_tool*(coordCOG_y^2 + coordCOG_x^2)]
	KDL::RotationalInertia displacedToolInertia = KDL::RotationalInertia(	m*(coordCOG.z()*coordCOG.z() + coordCOG.y()*coordCOG.y()),
																			m*(coordCOG.z()*coordCOG.z() + coordCOG.x()*coordCOG.x()),
																			m*(coordCOG.y()*coordCOG.y() + coordCOG.x()*coordCOG.x()),
																			-m*coordCOG.y()*coordCOG.x(),
																			-m*coordCOG.z()*coordCOG.x(),
																			-m*coordCOG.y()*coordCOG.z());
	
	KDL::RotationalInertia link7RInertia = KDL::RotationalInertia(XX7,YY7,ZZ7,XY7,XZ7,YZ7) + toolRInertia + displacedToolInertia;
	
	double XX7_tool = link7RInertia.data[0];
	double XY7_tool = link7RInertia.data[1];
	double XZ7_tool = link7RInertia.data[2];
	double YY7_tool = link7RInertia.data[4];
	double YZ7_tool = link7RInertia.data[5];
	double ZZ7_tool = link7RInertia.data[8]; 
	
}

void KukaKDL::setExternalToolWrench(KDL::Wrench W_ext){

	FX7 = W_ext.force.x();
	FY7 = W_ext.force.y();
	FZ7 = W_ext.force.z();
	CX7 = W_ext.torque.x();
	CY7 = W_ext.torque.x();
	CZ7 = W_ext.torque.x();
}

void KukaKDL::computeMassMatrix(){
	double S2, C2, S3, C3, S4, C4, S5, C5, S6, C6;
	double S7, C7, AS17, AS37, AJ117, AJ127, AJ137, AJ317, AJ327, AJ337;
	double AJA117, AJA217, AJA317, AJA337, XXP6, XYP6, XZP6, YYP6, YZP6, ZZP6;
	double MXP6, MYP6, MZP6, MP6, AS16, AS36, AJ116, AJ126, AJ136, AJ216;
	double AJ226, AJ316, AJ326, AJ336, AJA116, AJA216, AJA316, AJA336, XXP5, XYP5;
	double XZP5, YYP5, YZP5, ZZP5, MXP5, MYP5, MZP5, MP5, AS15, AS35;
	double AJ115, AJ125, AJ135, AJ215, AJ225, AJ315, AJ325, AJ335, AJA115, AJA215;
	double AJA315, AJA335, PAS115, PAS125, PAS325, PAS335, XXP4, XYP4, XZP4, YYP4;
	double YZP4, ZZP4, MXP4, MYP4, MZP4, MP4, AS14, AS34, AJ114, AJ124;
	double AJ134, AJ214, AJ224, AJ314, AJ324, AJ334, AJA114, AJA214, AJA314, AJA334;
	double XXP3, XYP3, XZP3, YYP3, YZP3, ZZP3, MXP3, MYP3, MZP3, MP3;
	double AS13, AS33, AJ113, AJ123, AJ133, AJ213, AJ223, AJ313, AJ323, AJ333;
	double AJA113, AJA213, AJA313, AJA333, PAS113, PAS123, PAS323, PAS333, XXP2, XYP2;
	double XZP2, YYP2, YZP2, ZZP2, AJ312, AJ322, AJ332, AJA332, ZZP1, EC12;
	double EC32, NC12, NC32, NC33, ED12, ED13, ED33, ND13, ND33, ND34;
	double EE12, EE32, NE12, NE32, EE13, NE13, NE33, EE14, EE34, NE14;
	double NE34, NE35, EF12, EF32, EF13, EF33, NF13, NF33, EF14, NF14;
	double NF34, EF15, EF35, NF15, NF35, NF36, EG12, EG32, EG13, EG33;
	double NG13, NG33, EG14, EG34, NG14, NG34, EG15, NG15, NG35, EG16;
	double EG36, NG16, NG36, NG37;
	double A11, A21, A31, A41, A51, A61, A71, A22, A32, A42, A52, A62, A72, A33;
	double A43, A53, A63, A73, A44, A54, A64, A74, A55, A65, A75, A66, A76, A77;
	


	// This code has been generated from the ouptut of the SYMORO+ program
	S2=sin(q(1));
	C2=cos(q(1));
	S3=sin(q(2));
	C3=cos(q(2));
	S4=sin(q(3));
	C4=cos(q(3));
	S5=sin(q(4));
	C5=cos(q(4));
	S6=sin(q(5));
	C6=cos(q(5));
	S7=sin(q(6));
	C7=cos(q(6));
	AS17=C7*MX7 - MY7*S7;
	AS37=-(C7*MY7) - MX7*S7;
	AJ117=C7*XX7_tool - S7*XY7_tool;
	AJ127=C7*XY7_tool - S7*YY7_tool;
	AJ137=C7*XZ7_tool - S7*YZ7_tool;
	AJ317=-(S7*XX7_tool) - C7*XY7_tool;
	AJ327=-(S7*XY7_tool) - C7*YY7_tool;
	AJ337=-(S7*XZ7_tool) - C7*YZ7_tool;
	AJA117=AJ117*C7 - AJ127*S7;
	AJA217=C7*XZ7_tool - S7*YZ7_tool;
	AJA317=AJ317*C7 - AJ327*S7;
	AJA337=-(AJ327*C7) - AJ317*S7;
	XXP6=AJA117 + XX6;
	XYP6=AJA217 + XY6;
	XZP6=AJA317 + XZ6;
	YYP6=YY6 + ZZ7_tool;
	YZP6=AJ337 + YZ6;
	ZZP6=AJA337 + ZZ6;
	MXP6=AS17 + MX6;
	MYP6=MY6 + MZ7;
	MZP6=AS37 + MZ6;
	MP6=M6 + M7;
	AS16=C6*MXP6 - MYP6*S6;
	AS36=C6*MYP6 + MXP6*S6;
	AJ116=C6*XXP6 - S6*(AJA217 + XY6);
	AJ126=C6*XYP6 - S6*YYP6;
	AJ136=C6*XZP6 - S6*YZP6;
	AJ216=-AJA317 - XZ6;
	AJ226=-AJ337 - YZ6;
	AJ316=S6*XXP6 + C6*(AJA217 + XY6);
	AJ326=S6*XYP6 + C6*YYP6;
	AJ336=S6*XZP6 + C6*YZP6;
	AJA116=AJ116*C6 - AJ126*S6;
	AJA216=AJ216*C6 - AJ226*S6;
	AJA316=AJ316*C6 - AJ326*S6;
	AJA336=AJ326*C6 + AJ316*S6;
	XXP5=AJA116 + XX5;
	XYP5=AJA216 + XY5;
	XZP5=AJA316 + XZ5;
	YYP5=YY5 + ZZP6;
	YZP5=-AJ336 + YZ5;
	ZZP5=AJA336 + ZZ5;
	MXP5=AS16 + MX5;
	MYP5=MY5 - MZP6;
	MZP5=AS36 + MZ5;
	MP5=M5 + MP6;
	AS15=C5*MXP5 - MYP5*S5;
	AS35=C5*MYP5 + MXP5*S5;
	AJ115=C5*XXP5 - S5*(AJA216 + XY5);
	AJ125=C5*XYP5 - S5*YYP5;
	AJ135=C5*XZP5 - S5*YZP5;
	AJ215=-AJA316 - XZ5;
	AJ225=AJ336 - YZ5;
	AJ315=S5*XXP5 + C5*(AJA216 + XY5);
	AJ325=S5*XYP5 + C5*YYP5;
	AJ335=S5*XZP5 + C5*YZP5;
	AJA115=AJ115*C5 - AJ125*S5;
	AJA215=AJ215*C5 - AJ225*S5;
	AJA315=AJ315*C5 - AJ325*S5;
	AJA335=AJ325*C5 + AJ315*S5;
	PAS115=-(MZP5*R5);
	PAS125=-(AS15*R5);
	PAS325=-(AS35*R5);
	PAS335=-(MZP5*R5);
	XXP4=AJA115 - 2*PAS115 + MP5*(R5*R5) + XX4;
	XYP4=AJA215 - PAS125 + XY4;
	XZP4=AJA315 + XZ4;
	YYP4=YY4 + ZZP5;
	YZP4=-AJ335 - PAS325 + YZ4;
	ZZP4=AJA335 - 2*PAS335 + MP5*(R5*R5) + ZZ4;
	MXP4=AS15 + MX4;
	MYP4=MY4 - MZP5 - MP5*R5;
	MZP4=AS35 + MZ4;
	MP4=M4 + MP5;
	AS14=C4*MXP4 - MYP4*S4;
	AS34=-(C4*MYP4) - MXP4*S4;
	AJ114=C4*XXP4 - S4*(AJA215 - PAS125 + XY4);
	AJ124=C4*XYP4 - S4*YYP4;
	AJ134=C4*XZP4 - S4*YZP4;
	AJ214=AJA315 + XZ4;
	AJ224=-AJ335 - PAS325 + YZ4;
	AJ314=-(S4*XXP4) - C4*(AJA215 - PAS125 + XY4);
	AJ324=-(S4*XYP4) - C4*YYP4;
	AJ334=-(S4*XZP4) - C4*YZP4;
	AJA114=AJ114*C4 - AJ124*S4;
	AJA214=AJ214*C4 - AJ224*S4;
	AJA314=AJ314*C4 - AJ324*S4;
	AJA334=-(AJ324*C4) - AJ314*S4;
	XXP3=AJA114 + XX3;
	XYP3=AJA214 + XY3;
	XZP3=AJA314 + XZ3;
	YYP3=YY3 + ZZP4;
	YZP3=AJ334 + YZ3;
	ZZP3=AJA334 + ZZ3;
	MXP3=AS14 + MX3;
	MYP3=MY3 + MZP4;
	MZP3=AS34 + MZ3;
	MP3=M3 + MP4;
	AS13=C3*MXP3 - MYP3*S3;
	AS33=-(C3*MYP3) - MXP3*S3;
	AJ113=C3*XXP3 - S3*(AJA214 + XY3);
	AJ123=C3*XYP3 - S3*YYP3;
	AJ133=C3*XZP3 - S3*YZP3;
	AJ213=AJA314 + XZ3;
	AJ223=AJ334 + YZ3;
	AJ313=-(S3*XXP3) - C3*(AJA214 + XY3);
	AJ323=-(S3*XYP3) - C3*YYP3;
	AJ333=-(S3*XZP3) - C3*YZP3;
	AJA113=AJ113*C3 - AJ123*S3;
	AJA213=AJ213*C3 - AJ223*S3;
	AJA313=AJ313*C3 - AJ323*S3;
	AJA333=-(AJ323*C3) - AJ313*S3;
	PAS113=-(MZP3*R3);
	PAS123=AS13*R3;
	PAS323=AS33*R3;
	PAS333=-(MZP3*R3);
	XXP2=AJA113 - 2*PAS113 + MP3*(R3*R3) + XX2;
	XYP2=AJA213 - PAS123 + XY2;
	XZP2=AJA313 + XZ2;
	YYP2=YY2 + ZZP3;
	YZP2=AJ333 - PAS323 + YZ2;
	ZZP2=AJA333 - 2*PAS333 + MP3*(R3*R3) + ZZ2;
	AJ312=S2*XXP2 + C2*(AJA213 - PAS123 + XY2);
	AJ322=S2*XYP2 + C2*YYP2;
	AJ332=S2*XZP2 + C2*YZP2;
	AJA332=AJ322*C2 + AJ312*S2;
	ZZP1=AJA332 + ZZ1;
	EC12=-(C3*MYP3) - MXP3*S3;
	EC32=-(C3*MXP3) + MYP3*S3;
	NC12=AJ133 + EC32*R3;
	NC32=AJ333 - EC12*R3;
	NC33=NC12*S2 + C2*ZZP3;
	ED12=-(C4*MYP4) - MXP4*S4;
	ED13=C3*ED12;
	ED33=-(ED12*S3);
	ND13=AJ134*C3 + ED33*R3 - S3*ZZP4;
	ND33=-(ED13*R3) - AJ134*S3 - C3*ZZP4;
	ND34=AJ334*C2 + ND13*S2;
	EE12=-(C5*MYP5) - MXP5*S5;
	EE32=C5*MXP5 - MYP5*S5;
	NE12=AJ135 - EE32*R5;
	NE32=AJ335 + EE12*R5;
	EE13=C4*EE12;
	NE13=C4*NE12 + S4*ZZP5;
	NE33=-(NE12*S4) + C4*ZZP5;
	EE14=C3*EE13 - EE32*S3;
	EE34=-(C3*EE32) - EE13*S3;
	NE14=C3*NE13 + EE34*R3 - NE32*S3;
	NE34=-(C3*NE32) - EE14*R3 - NE13*S3;
	NE35=C2*NE33 + NE14*S2;
	EF12=-(C6*MYP6) - MXP6*S6;
	EF32=C6*MXP6 - MYP6*S6;
	EF13=C5*EF12;
	EF33=EF12*S5;
	NF13=AJ136*C5 - EF33*R5 + S5*ZZP6;
	NF33=EF13*R5 + AJ136*S5 - C5*ZZP6;
	EF14=C4*EF13 + EF32*S4;
	NF14=C4*NF13 + AJ336*S4;
	NF34=AJ336*C4 - NF13*S4;
	EF15=C3*EF14 - EF33*S3;
	EF35=-(C3*EF33) - EF14*S3;
	NF15=C3*NF14 + EF35*R3 - NF33*S3;
	NF35=-(C3*NF33) - EF15*R3 - NF14*S3;
	NF36=C2*NF34 + NF15*S2;
	EG12=-(C7*MY7) - MX7*S7;
	EG32=-(C7*MX7) + MY7*S7;
	EG13=C6*EG12;
	EG33=EG12*S6;
	NG13=AJ137*C6 - S6*ZZ7_tool;
	NG33=AJ137*S6 + C6*ZZ7_tool;
	EG14=C5*EG13 + EG32*S5;
	EG34=-(C5*EG32) + EG13*S5;
	NG14=C5*NG13 - EG34*R5 + AJ337*S5;
	NG34=-(AJ337*C5) + EG14*R5 + NG13*S5;
	EG15=C4*EG14 + EG33*S4;
	NG15=C4*NG14 + NG33*S4;
	NG35=C4*NG33 - NG14*S4;
	EG16=C3*EG15 - EG34*S3;
	EG36=-(C3*EG34) - EG15*S3;
	NG16=C3*NG15 + EG36*R3 - NG34*S3;
	NG36=-(C3*NG34) - EG16*R3 - NG15*S3;
	NG37=C2*NG35 + NG16*S2;
	A11=IA1 + ZZP1;
	A21=AJ332;
	A31=NC33;
	A41=ND34;
	A51=NE35;
	A61=NF36;
	A71=NG37;
	A22=IA2 + ZZP2;
	A32=NC32;
	A42=ND33;
	A52=NE34;
	A62=NF35;
	A72=NG36;
	A33=IA3 + ZZP3;
	A43=AJ334;
	A53=NE33;
	A63=NF34;
	A73=NG35;
	A44=IA4 + ZZP4;
	A54=NE32;
	A64=NF33;
	A74=NG34;
	A55=IA5 + ZZP5;
	A65=AJ336;
	A75=NG33;
	A66=IA6 + ZZP6;
	A76=AJ337;
	A77=IA7 + ZZ7_tool;

	massMatrix.data << 	A11, A21, A31, A41, A51, A61, A71,
							A21, A22, A32, A42, A52, A62, A72,
							A31, A32, A33, A43, A53, A63, A73,
							A41, A42, A43, A44, A54, A64, A74,
							A51, A52, A53, A54, A55, A65, A75,
							A61, A62, A63, A64, A65, A66, A76,
							A71, A72, A73, A74, A75, A76, A77;
	
}

void KukaKDL::computeCorioCentriGravTorque(){
	
	double S1, C1, S2, C2, S3, C3, S4, C4, S5, C5;
	double S6, C6, S7, C7, VP11, VP21, WI12, WI22, WP12, WP22;
	double DV112, DV222, DV332, DV122, DV132, DV232, U132, U222, U232, U312;
	double U322, VP12, VP22, PIS12, PIS22, PIS32, No12, No22, No32, WI13;
	double WI23, W33, WP13, WP23, DV113, DV223, DV333, DV123, DV133, DV233;
	double U113, U123, U133, U213, U223, U233, U313, U323, VSP13, VSP23;
	double VSP33, VP13, VP23, F13, F23, PIS13, PIS23, PIS33, No13, No23;
	double No33, WI14, WI24, W34, WP14, WP24, DV114, DV224, DV334, DV124;
	double DV134, DV234, U114, U124, U134, U214, U224, U234, U314, U324;
	double U334, VP14, VP24, F14, F24, F34, PIS14, PIS24, PIS34, No14;
	double No24, No34, WI15, WI25, W35, WP15, WP25, DV115, DV225, DV335;
	double DV125, DV135, DV235, U115, U125, U135, U215, U225, U235, U315;
	double U325, U335, VSP15, VSP25, VSP35, VP15, VP25, F15, F25, F35;
	double PIS15, PIS25, PIS35, No15, No25, No35, WI16, WI26, W36, WP16;
	double WP26, DV116, DV226, DV336, DV126, DV136, DV236, U116, U126, U136;
	double U216, U226, U236, U316, U326, U336, VP16, VP26, F16, F26;
	double F36, PIS16, PIS26, PIS36, No16, No26, No36, WI17, WI27, W37;
	double WP17, WP27, DV117, DV227, DV337, DV127, DV137, DV237, U117, U127;
	double U137, U217, U227, U237, U317, U327, U337, VP17, VP27, F17;
	double F27, F37, PIS17, PIS27, PIS37, No17, No27, No37, E17, E27;
	double E37, N17, N27, N37, FDI17, FDI37, E16, E26, E36, N16;
	double N26, N36, FDI16, FDI36, E15, E25, E35, N15, N25, N35;
	double FDI15, FDI35, E14, E24, E34, N14, N24, N34, FDI14, E13;
	double E23, N13, N23, N33, FDI13, FDI33, N12, N22, N32, N31;

	S1=sin(q(0));
	C1=cos(q(0));
	S2=sin(q(1));
	C2=cos(q(1));
	S3=sin(q(2));
	C3=cos(q(2));
	S4=sin(q(3));
	C4=cos(q(3));
	S5=sin(q(4));
	C5=cos(q(4));
	S6=sin(q(5));
	C6=cos(q(5));
	S7=sin(q(6));
	C7=cos(q(6));
	VP11=-(C1*GX) - GY*S1;
	VP21=-(C1*GY) + GX*S1;
	WI12=qd(0)*S2;
	WI22=C2*qd(0);
	WP12=qd(1)*WI22;
	WP22=-(qd(1)*WI12);
	DV112=-(WI12*WI12);
	DV222=-(WI22*WI22);
	DV332=-(qd(1)*qd(1));
	DV122=WI12*WI22;
	DV132=qd(1)*WI12;
	DV232=qd(1)*WI22;
	U132=DV132 + WP22;
	U222=DV112 + DV332;
	U232=DV232 - WP12;
	U312=DV132 - WP22;
	U322=DV232 + WP12;
	VP12=-(GZ*S2) + C2*VP11;
	VP22=-(C2*GZ) - S2*VP11;
	PIS12=-YY2 + ZZ2;
	PIS22=XX2 - ZZ2;
	PIS32=-XX2 + YY2;
	No12=DV232*PIS12 + WP12*XX2 - U312*XY2 + DV122*XZ2 + (-DV222 + DV332)*YZ2;
	No22=DV132*PIS22 + U322*XY2 + (DV112 - DV332)*XZ2 + WP22*YY2 - DV122*YZ2;
	No32=DV122*PIS32 + (-DV112 + DV222)*XY2 - U232*XZ2 + U132*YZ2;
	WI13=-(qd(1)*S3) + C3*WI12;
	WI23=-(C3*qd(1)) - S3*WI12;
	W33=qd(2) + WI22;
	WP13=qd(2)*WI23 + C3*WP12;
	WP23=-(qd(2)*WI13) - S3*WP12;
	DV113=-(WI13*WI13);
	DV223=-(WI23*WI23);
	DV333=-(W33*W33);
	DV123=WI13*WI23;
	DV133=W33*WI13;
	DV233=W33*WI23;
	U113=DV223 + DV333;
	U123=DV123 - WP22;
	U133=DV133 + WP23;
	U213=DV123 + WP22;
	U223=DV113 + DV333;
	U233=DV233 - WP13;
	U313=DV133 - WP23;
	U323=DV233 + WP13;
	VSP13=DV122*R3 + VP12;
	VSP23=R3*U222 + VP22;
	VSP33=R3*U322 - VP21;
	VP13=C3*VSP13 - S3*VSP33;
	VP23=-(S3*VSP13) - C3*VSP33;
	F13=MX3*U113 + MY3*U123 + MZ3*U133 + M3*VP13;
	F23=MX3*U213 + MY3*U223 + MZ3*U233 + M3*VP23;
	PIS13=-YY3 + ZZ3;
	PIS23=XX3 - ZZ3;
	PIS33=-XX3 + YY3;
	No13=DV233*PIS13 + WP13*XX3 - U313*XY3 + U213*XZ3 + (-DV223 + DV333)*YZ3;
	No23=DV133*PIS23 + U323*XY3 + (DV113 - DV333)*XZ3 + WP23*YY3 - U123*YZ3;
	No33=DV123*PIS33 + (-DV113 + DV223)*XY3 - U233*XZ3 + U133*YZ3 + WP22*ZZ3;
	WI14=-(S4*W33) + C4*WI13;
	WI24=-(C4*W33) - S4*WI13;
	W34=qd(3) + WI23;
	WP14=qd(3)*WI24 + C4*WP13 - S4*WP22;
	WP24=-(qd(3)*WI14) - S4*WP13 - C4*WP22;
	DV114=-(WI14*WI14);
	DV224=-(WI24*WI24);
	DV334=-(W34*W34);
	DV124=WI14*WI24;
	DV134=W34*WI14;
	DV234=W34*WI24;
	U114=DV224 + DV334;
	U124=DV124 - WP23;
	U134=DV134 + WP24;
	U214=DV124 + WP23;
	U224=DV114 + DV334;
	U234=DV234 - WP14;
	U314=DV134 - WP24;
	U324=DV234 + WP14;
	U334=DV114 + DV224;
	VP14=C4*VP13 - S4*VSP23;
	VP24=-(S4*VP13) - C4*VSP23;
	F14=MX4*U114 + MY4*U124 + MZ4*U134 + M4*VP14;
	F24=MX4*U214 + MY4*U224 + MZ4*U234 + M4*VP24;
	F34=MX4*U314 + MY4*U324 + MZ4*U334 + M4*VP23;
	PIS14=-YY4 + ZZ4;
	PIS24=XX4 - ZZ4;
	PIS34=-XX4 + YY4;
	No14=DV234*PIS14 + WP14*XX4 - U314*XY4 + U214*XZ4 + (-DV224 + DV334)*YZ4;
	No24=DV134*PIS24 + U324*XY4 + (DV114 - DV334)*XZ4 + WP24*YY4 - U124*YZ4;
	No34=DV124*PIS34 + (-DV114 + DV224)*XY4 - U234*XZ4 + U134*YZ4 + WP23*ZZ4;
	WI15=S5*W34 + C5*WI14;
	WI25=C5*W34 - S5*WI14;
	W35=qd(3) - WI24;
	WP15=qd(3)*WI25 + C5*WP14 + S5*WP23;
	WP25=-(qd(3)*WI15) - S5*WP14 + C5*WP23;
	DV115=-(WI15*WI15);
	DV225=-(WI25*WI25);
	DV335=-(W35*W35);
	DV125=WI15*WI25;
	DV135=W35*WI15;
	DV235=W35*WI25;
	U115=DV225 + DV335;
	U125=DV125 + WP24;
	U135=DV135 + WP25;
	U215=DV125 - WP24;
	U225=DV115 + DV335;
	U235=DV235 - WP15;
	U315=DV135 - WP25;
	U325=DV235 + WP15;
	U335=DV115 + DV225;
	VSP15=-(R5*U124) + VP14;
	VSP25=-(R5*U224) + VP24;
	VSP35=-(R5*U324) + VP23;
	VP15=C5*VSP15 + S5*VSP35;
	VP25=-(S5*VSP15) + C5*VSP35;
	F15=MX5*U115 + MY5*U125 + MZ5*U135 + M5*VP15;
	F25=MX5*U215 + MY5*U225 + MZ5*U235 + M5*VP25;
	F35=MX5*U315 + MY5*U325 + MZ5*U335 - M5*VSP25;
	PIS15=-YY5 + ZZ5;
	PIS25=XX5 - ZZ5;
	PIS35=-XX5 + YY5;
	No15=DV235*PIS15 + WP15*XX5 - U315*XY5 + U215*XZ5 + (-DV225 + DV335)*YZ5;
	No25=DV135*PIS25 + U325*XY5 + (DV115 - DV335)*XZ5 + WP25*YY5 - U125*YZ5;
	No35=DV125*PIS35 + (-DV115 + DV225)*XY5 - U235*XZ5 + U135*YZ5 - WP24*ZZ5;
	WI16=S6*W35 + C6*WI15;
	WI26=C6*W35 - S6*WI15;
	W36=qd(5) - WI25;
	WP16=qd(5)*WI26 + C6*WP15 - S6*WP24;
	WP26=-(qd(5)*WI16) - S6*WP15 - C6*WP24;
	DV116=-(WI16*WI16);
	DV226=-(WI26*WI26);
	DV336=-(W36*W36);
	DV126=WI16*WI26;
	DV136=W36*WI16;
	DV236=W36*WI26;
	U116=DV226 + DV336;
	U126=DV126 + WP25;
	U136=DV136 + WP26;
	U216=DV126 - WP25;
	U226=DV116 + DV336;
	U236=DV236 - WP16;
	U316=DV136 - WP26;
	U326=DV236 + WP16;
	U336=DV116 + DV226;
	VP16=C6*VP15 - S6*VSP25;
	VP26=-(S6*VP15) - C6*VSP25;
	F16=MX6*U116 + MY6*U126 + MZ6*U136 + M6*VP16;
	F26=MX6*U216 + MY6*U226 + MZ6*U236 + M6*VP26;
	F36=MX6*U316 + MY6*U326 + MZ6*U336 - M6*VP25;
	PIS16=-YY6 + ZZ6;
	PIS26=XX6 - ZZ6;
	PIS36=-XX6 + YY6;
	No16=DV236*PIS16 + WP16*XX6 - U316*XY6 + U216*XZ6 + (-DV226 + DV336)*YZ6;
	No26=DV136*PIS26 + U326*XY6 + (DV116 - DV336)*XZ6 + WP26*YY6 - U126*YZ6;
	No36=DV126*PIS36 + (-DV116 + DV226)*XY6 - U236*XZ6 + U136*YZ6 - WP25*ZZ6;
	WI17=-(S7*W36) + C7*WI16;
	WI27=-(C7*W36) - S7*WI16;
	W37=qd(6) + WI26;
	WP17=qd(6)*WI27 + C7*WP16 + S7*WP25;
	WP27=-(qd(6)*WI17) - S7*WP16 + C7*WP25;
	DV117=-(WI17*WI17);
	DV227=-(WI27*WI27);
	DV337=-(W37*W37);
	DV127=WI17*WI27;
	DV137=W37*WI17;
	DV237=W37*WI27;
	U117=DV227 + DV337;
	U127=DV127 - WP26;
	U137=DV137 + WP27;
	U217=DV127 + WP26;
	U227=DV117 + DV337;
	U237=DV237 - WP17;
	U317=DV137 - WP27;
	U327=DV237 + WP17;
	U337=DV117 + DV227;
	VP17=C7*VP16 + S7*VP25;
	VP27=-(S7*VP16) + C7*VP25;
	F17=MX7*U117 + MY7*U127 + MZ7*U137 + M7*VP17;
	F27=MX7*U217 + MY7*U227 + MZ7*U237 + M7*VP27;
	F37=MX7*U317 + MY7*U327 + MZ7*U337 + M7*VP26;
	PIS17=-YY7_tool + ZZ7_tool;
	PIS27=XX7_tool - ZZ7_tool;
	PIS37=-XX7_tool + YY7_tool;
	No17=DV237*PIS17 + WP17*XX7_tool - U317*XY7_tool + U217*XZ7_tool + (-DV227 + DV337)*YZ7_tool;
	No27=DV137*PIS27 + U327*XY7_tool + (DV117 - DV337)*XZ7_tool + WP27*YY7_tool - U127*YZ7_tool;
	No37=DV127*PIS37 + (-DV117 + DV227)*XY7_tool - U237*XZ7_tool + U137*YZ7_tool + WP26*ZZ7_tool;
	E17=F17 + FX7;
	E27=F27 + FY7;
	E37=F37 + FZ7;
	N17=CX7 + No17 + MY7*VP26 - MZ7*VP27;
	N27=CY7 + No27 + MZ7*VP17 - MX7*VP26;
	N37=CZ7 + No37 - MY7*VP17 + MX7*VP27;
	FDI17=C7*E17 - E27*S7;
	FDI37=-(C7*E27) - E17*S7;
	E16=F16 + FDI17;
	E26=E37 + F26;
	E36=F36 + FDI37;
	N16=C7*N17 + No16 - N27*S7 - MY6*VP25 - MZ6*VP26;
	N26=N37 + No26 + MZ6*VP16 + MX6*VP25;
	N36=-(C7*N27) + No36 - N17*S7 - MY6*VP16 + MX6*VP26;
	FDI16=C6*E16 - E26*S6;
	FDI36=C6*E26 + E16*S6;
	E15=F15 + FDI16;
	E25=-E36 + F25;
	E35=F35 + FDI36;
	N15=C6*N16 + No15 - N26*S6 - MZ5*VP25 - MY5*VSP25;
	N25=-N36 + No25 + MZ5*VP15 + MX5*VSP25;
	N35=C6*N26 + No35 + N16*S6 - MY5*VP15 + MX5*VP25;
	FDI15=C5*E15 - E25*S5;
	FDI35=C5*E25 + E15*S5;
	E14=F14 + FDI15;
	E24=-E35 + F24;
	E34=F34 + FDI35;
	N14=C5*N15 + No14 - FDI35*R5 - N25*S5 + MY4*VP23 - MZ4*VP24;
	N24=-N35 + No24 + MZ4*VP14 - MX4*VP23;
	N34=C5*N25 + No34 + FDI15*R5 + N15*S5 - MY4*VP14 + MX4*VP24;
	FDI14=C4*E14 - E24*S4;
	E13=F13 + FDI14;
	E23=E34 + F23;
	N13=C4*N14 + No13 - N24*S4 - MZ3*VP23 + MY3*VSP23;
	N23=N34 + No23 + MZ3*VP13 - MX3*VSP23;
	N33=-(C4*N24) + No33 - N14*S4 - MY3*VP13 + MX3*VP23;
	FDI13=C3*E13 - E23*S3;
	FDI33=-(C3*E23) - E13*S3;
	N12=C3*N13 + No12 + FDI33*R3 - N23*S3 - MY2*VP21 - MZ2*VP22;
	N22=N33 + No22 + MZ2*VP12 + MX2*VP21;
	N32=-(C3*N23) + No32 - FDI13*R3 - N13*S3 - MY2*VP12 + MX2*VP22;
	N31=C2*N22 + N12*S2 - MY1*VP11 + MX1*VP21;
	
	corioCentriGravTorque.data << N31, N32, N33, N34, N35, N36, N37;
}

void KukaKDL::computeGravityTorque(){
	
	double S1, C1, S2, C2, S3, C3, S4, C4, S5, C5;
	double S6, C6, S7, C7, AS17, AS37, MXP6, MYP6, MZP6, MP6;
	double AS16, AS36, MXP5, MYP5, MZP5, MP5, AS15, AS35, MXP4, MYP4;
	double MZP4, MP4, AS14, AS34, MXP3, MYP3, MZP3, MP3, AS13, AS33;
	double MXP2, MYP2, MZP2, AS12, MXP1, MYP1, PMA0112, PMA0122, PMA0212, PMA0222;
	double PMA0113, PMA0123, PMA0213, PMA0223, PMA0313, PMA0323, PMA0114, PMA0124, PMA0214, PMA0224;
	double PMA0314, PMA0324, PMA0115, PMA0125, PMA0215, PMA0225, PMA0315, PMA0325, PMA0116, PMA0126;
	double PMA0216, PMA0226, PMA0316, PMA0326, PMA0117, PMA0127, PMA0217, PMA0227, PMA0317, PMA0327;
	double VQ11, VQ21, VQ12, VQ22, VQ32, VQ13, VQ23, VQ33, VQ14, VQ24;
	double VQ34, VQ15, VQ25, VQ35, VQ16, VQ26, VQ36, VQ17, VQ27, VQ37;
	double Q1, Q2, Q3, Q4, Q5, Q6, Q7;

	S1=sin(q(0));
	C1=cos(q(0));
	S2=sin(q(1));
	C2=cos(q(1));
	S3=sin(q(2));
	C3=cos(q(2));
	S4=sin(q(3));
	C4=cos(q(3));
	S5=sin(q(4));
	C5=cos(q(4));
	S6=sin(q(5));
	C6=cos(q(5));
	S7=sin(q(6));
	C7=cos(q(6));
	AS17=C7*MX7 - MY7*S7;
	AS37=-(C7*MY7) - MX7*S7;
	MXP6=AS17 + MX6;
	MYP6=MY6 + MZ7;
	MZP6=AS37 + MZ6;
	MP6=M6 + M7;
	AS16=C6*MXP6 - MYP6*S6;
	AS36=C6*MYP6 + MXP6*S6;
	MXP5=AS16 + MX5;
	MYP5=MY5 - MZP6;
	MZP5=AS36 + MZ5;
	MP5=M5 + MP6;
	AS15=C5*MXP5 - MYP5*S5;
	AS35=C5*MYP5 + MXP5*S5;
	MXP4=AS15 + MX4;
	MYP4=MY4 - MZP5 - MP5*R5;
	MZP4=AS35 + MZ4;
	MP4=M4 + MP5;
	AS14=C4*MXP4 - MYP4*S4;
	AS34=-(C4*MYP4) - MXP4*S4;
	MXP3=AS14 + MX3;
	MYP3=MY3 + MZP4;
	MZP3=AS34 + MZ3;
	MP3=M3 + MP4;
	AS13=C3*MXP3 - MYP3*S3;
	AS33=-(C3*MYP3) - MXP3*S3;
	MXP2=AS13 + MX2;
	MYP2=MY2 + MZP3 + MP3*R3;
	MZP2=AS33 + MZ2;
	AS12=C2*MXP2 - MYP2*S2;
	MXP1=AS12 + MX1;
	MYP1=MY1 - MZP2;
	PMA0112=C1*C2;
	PMA0122=-(C1*S2);
	PMA0212=C2*S1;
	PMA0222=-(S1*S2);
	PMA0113=C3*PMA0112 - S1*S3;
	PMA0123=-(C3*S1) - PMA0112*S3;
	PMA0213=C3*PMA0212 + C1*S3;
	PMA0223=C1*C3 - PMA0212*S3;
	PMA0313=C3*S2;
	PMA0323=-(S2*S3);
	PMA0114=C4*PMA0113 - PMA0122*S4;
	PMA0124=-(C4*PMA0122) - PMA0113*S4;
	PMA0214=C4*PMA0213 - PMA0222*S4;
	PMA0224=-(C4*PMA0222) - PMA0213*S4;
	PMA0314=C4*PMA0313 - C2*S4;
	PMA0324=-(C2*C4) - PMA0313*S4;
	PMA0115=C5*PMA0114 + PMA0123*S5;
	PMA0125=C5*PMA0123 - PMA0114*S5;
	PMA0215=C5*PMA0214 + PMA0223*S5;
	PMA0225=C5*PMA0223 - PMA0214*S5;
	PMA0315=C5*PMA0314 + PMA0323*S5;
	PMA0325=C5*PMA0323 - PMA0314*S5;
	PMA0116=C6*PMA0115 - PMA0124*S6;
	PMA0126=-(C6*PMA0124) - PMA0115*S6;
	PMA0216=C6*PMA0215 - PMA0224*S6;
	PMA0226=-(C6*PMA0224) - PMA0215*S6;
	PMA0316=C6*PMA0315 - PMA0324*S6;
	PMA0326=-(C6*PMA0324) - PMA0315*S6;
	PMA0117=C7*PMA0116 + PMA0125*S7;
	PMA0127=C7*PMA0125 - PMA0116*S7;
	PMA0217=C7*PMA0216 + PMA0225*S7;
	PMA0227=C7*PMA0225 - PMA0216*S7;
	PMA0317=C7*PMA0316 + PMA0325*S7;
	PMA0327=C7*PMA0325 - PMA0316*S7;
	VQ11=-(C1*MYP1) - MXP1*S1;
	VQ21=C1*MXP1 - MYP1*S1;
	VQ12=-(MYP2*PMA0112) + MXP2*PMA0122;
	VQ22=-(MYP2*PMA0212) + MXP2*PMA0222;
	VQ32=C2*MXP2 - MYP2*S2;
	VQ13=-(MYP3*PMA0113) + MXP3*PMA0123;
	VQ23=-(MYP3*PMA0213) + MXP3*PMA0223;
	VQ33=-(MYP3*PMA0313) + MXP3*PMA0323;
	VQ14=-(MYP4*PMA0114) + MXP4*PMA0124;
	VQ24=-(MYP4*PMA0214) + MXP4*PMA0224;
	VQ34=-(MYP4*PMA0314) + MXP4*PMA0324;
	VQ15=-(MYP5*PMA0115) + MXP5*PMA0125;
	VQ25=-(MYP5*PMA0215) + MXP5*PMA0225;
	VQ35=-(MYP5*PMA0315) + MXP5*PMA0325;
	VQ16=-(MYP6*PMA0116) + MXP6*PMA0126;
	VQ26=-(MYP6*PMA0216) + MXP6*PMA0226;
	VQ36=-(MYP6*PMA0316) + MXP6*PMA0326;
	VQ17=-(MY7*PMA0117) + MX7*PMA0127;
	VQ27=-(MY7*PMA0217) + MX7*PMA0227;
	VQ37=-(MY7*PMA0317) + MX7*PMA0327;
	Q1=-(GX*VQ11) - GY*VQ21;
	Q2=-(GX*VQ12) - GY*VQ22 - GZ*VQ32;
	Q3=-(GX*VQ13) - GY*VQ23 - GZ*VQ33;
	Q4=-(GX*VQ14) - GY*VQ24 - GZ*VQ34;
	Q5=-(GX*VQ15) - GY*VQ25 - GZ*VQ35;
	Q6=-(GX*VQ16) - GY*VQ26 - GZ*VQ36;
	Q7=-(GX*VQ17) - GY*VQ27 - GZ*VQ37;
	
	gravityTorque.data << Q1, Q2, Q3, Q4, Q5, Q6, Q7;
	
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

void KukaKDL::computeMassMatrixFromKDL(){
    dynModelSolver->JntToMass(q,massMatrixFromKDL);
}

void KukaKDL::computeCorioCentriTorqueFromKDL(){
    dynModelSolver->JntToCoriolis(q,qd,corioCentriTorqueFromKDL);
}

void KukaKDL::computeGravityTorqueFromKDL(){
    dynModelSolver->JntToGravity(q,gravityTorqueFromKDL);
}

void KukaKDL::outdate(){
    inertiaMatrixOutdated = true;
    corioCentriTorqueOutdated = true;
    gravityOutdated = true;
}

