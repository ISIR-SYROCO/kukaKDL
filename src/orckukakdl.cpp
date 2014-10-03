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

struct OrcKukaKDL::Pimpl{
    public:
        //=============== General Variables ================//
        int                                      nbSeg;
        Eigen::VectorXd                          actuatedDofs;
        Eigen::VectorXd                          lowerLimits;
        Eigen::VectorXd                          upperLimits;
        Eigen::VectorXd                          q;
        Eigen::VectorXd                          dq;
        Eigen::Displacementd                     Hroot;
        Eigen::Twistd                            Troot;
        Eigen::VectorXd                          alldq;
        bool                                     isFreeFlyer;
        int                                      nbDofs;
        Eigen::Matrix<double,6,1>                gravity_dtwist;

        //=============== Dynamic Variables ================//
        Eigen::MatrixXd                          M;
        Eigen::MatrixXd                          Minv;
        Eigen::MatrixXd                          B;
        Eigen::VectorXd                          n;
        Eigen::VectorXd                          l;
        Eigen::VectorXd                          g;

        //================= CoM Variables ==================//
        double                                   total_mass;
        Eigen::Vector3d                          comPosition;
        Eigen::Vector3d                          comVelocity;
        Eigen::Vector3d                          comJdotQdot;
        Eigen::Matrix<double,3,Eigen::Dynamic>   comJacobian;
        Eigen::Matrix<double,3,Eigen::Dynamic>   comJacobianDot;

        //=============== Segments Variables ===============//
        std::vector< double >                                   segMass;
        std::vector< Eigen::Vector3d >                          segCoM;
        std::vector< Eigen::Matrix<double,6,6> >                segMassMatrix;
        std::vector< Eigen::Vector3d >                          segMomentsOfInertia;
        std::vector< Eigen::Rotation3d >                        segInertiaAxes;
        std::vector< Eigen::Displacementd >                     segPosition;
        std::vector< Eigen::Twistd >                            segVelocity;
        std::vector< Eigen::Matrix<double,6,Eigen::Dynamic> >   segJacobian;
        std::vector< Eigen::Matrix<double,6,Eigen::Dynamic> >   segJdot;
        std::vector< Eigen::Matrix<double,6,Eigen::Dynamic> >   segJointJacobian;
        std::vector< Eigen::Twistd >                            segJdotQdot;

        //================ Other Variables =================//
        std::map< std::string, int >             segIndexFromName;
        std::vector< std::string >               segNameFromIndex;

        KukaKDL kdlmodel;
        
        Pimpl():
        nbSeg(9),
        actuatedDofs(7),
        lowerLimits(7),
        upperLimits(7),
        q(7),
        dq(7),            
        Hroot(0, 0, 0),         
        Troot(0, 0, 0, 0, 0, 0),         
        alldq(7),         
        isFreeFlyer(false),   
        nbDofs(7),        
        gravity_dtwist(Eigen::Matrix<double,6,1>::Zero()),
        M(7, 7),   
        Minv(7, 7),
        B(7, 7),   
        n(7),   
        l(7),   
        g(7),   
        total_mass(0.0),    
        comPosition(0, 0, 0),   
        comVelocity(0, 0, 0),   
        comJdotQdot(0, 0, 0),   
        comJacobian(3, 7),   
        comJacobianDot(3, 7),
        segMass(9, double(0)),            
        segCoM(9, Eigen::Vector3d(0,0,0)),             
        segMassMatrix(9, Eigen::Matrix<double,6,6>()),      
        segMomentsOfInertia(9, Eigen::Vector3d(0,0,0)),
        segInertiaAxes(9, Eigen::Rotation3d()),     
        segPosition(9, Eigen::Displacementd(0,0,0)),        
        segVelocity(9, Eigen::Twistd(0,0,0,0,0,0)),        
        segJacobian(9, Eigen::Matrix<double,6,Eigen::Dynamic>(6,7)),        
        segJdot(9, Eigen::Matrix<double,6,Eigen::Dynamic>(6,7)),            
        segJointJacobian(9, Eigen::Matrix<double,6,Eigen::Dynamic>(6,7)),   
        segJdotQdot(9, Eigen::Twistd(0,0,0,0,0,0)) ,        
        segIndexFromName(),
        segNameFromIndex(9, std::string(""))
        {
            for(int i=0; i<nbSeg; ++i){
                double m = kdlmodel.getSegment(i).getInertia().getMass();
                total_mass += m;
                segMass[i] = m;
            }
            actuatedDofs = kdlmodel.getActuatedDofs();
            lowerLimits = kdlmodel.getJointLowerLimits();
            upperLimits = kdlmodel.getJointUpperLimits();
            gravity_dtwist   << 0, 0 ,0, 0, 0, -9.80665;
            B.setZero();
            B.diagonal() = Eigen::VectorXd::Constant(nbDofs, 0.001);
            l.setZero();
            comPosition.setZero();
            comVelocity.setZero();
            comJdotQdot.setZero();
            comJacobian.setZero();
            comJacobianDot.setZero();
            for(int i=0; i<8; ++i){
                segCoM[i].setZero();
                segMassMatrix[i].setZero();
                segMomentsOfInertia[i].setZero();
                segJdot[i].setZero();
            }

            segNameFromIndex.push_back("base");
            segNameFromIndex.push_back("00");
            segNameFromIndex.push_back("01");
            segNameFromIndex.push_back("02");
            segNameFromIndex.push_back("03");
            segNameFromIndex.push_back("04");
            segNameFromIndex.push_back("05");
            segNameFromIndex.push_back("06");
            segNameFromIndex.push_back("07");
            
            segIndexFromName["base"] = 0;
            segIndexFromName["00"] = 1;
            segIndexFromName["01"] = 2;
            segIndexFromName["02"] = 3;
            segIndexFromName["03"] = 4;
            segIndexFromName["04"] = 5;
            segIndexFromName["05"] = 6;
            segIndexFromName["06"] = 7;
            segIndexFromName["07"] = 8;

        }

        ~Pimpl(){
        }

        int nbSegments()
        {
            return nbSeg;
        }

        const Eigen::VectorXd& getActuatedDofs()
        {
            return actuatedDofs;
        }

        const Eigen::VectorXd& getJointLowerLimits()
        {
            return lowerLimits;
        }

        const Eigen::VectorXd& getJointUpperLimits()
        {
            return upperLimits;
        }

        const Eigen::VectorXd& getJointPositions()
        {
            return q;
        }

        const Eigen::VectorXd& getJointVelocities()
        {
            return dq;
        }

        const Eigen::Displacementd& getFreeFlyerPosition()
        {
            return Hroot;
        }

        const Eigen::Twistd& getFreeFlyerVelocity()
        {
            return Troot;
        }

        const Eigen::MatrixXd& getInertiaMatrix()
        {
            if (kdlmodel.inertiaMatrixOutdated())
            {
                M.setZero();
                M = kdlmodel.getInertiaMatrix().data;
            }
            return M;
        }

        const Eigen::MatrixXd& getInertiaMatrixInverse()
        {
            if (kdlmodel.inertiaMatrixOutdated())
            {
                M = kdlmodel.getInertiaMatrix().data;
                Minv = M.inverse();
            }
            Minv = M.inverse();
            return Minv;
        }

        const Eigen::MatrixXd& getDampingMatrix()
        {
            return B;
        }

        const Eigen::VectorXd& getNonLinearTerms()
        {
            if (kdlmodel.corioCentriTorqueOutdated())
            {
                n.setZero();
                n = kdlmodel.getNonLinearTerms().data;
            }
            return n;
        }

        const Eigen::VectorXd& getLinearTerms()
        {
            return l;
        }

        const Eigen::VectorXd& getGravityTerms()
        {
            if (kdlmodel.gravityOutdated())
            {
                g.setZero();
                g = kdlmodel.getGravityTerms().data;
            }
            return g;
        }

        double getMass()
        {
            return total_mass;
        }

        const Eigen::Vector3d& getCoMPosition()
        {
            return comPosition;
        }

        const Eigen::Vector3d& getCoMVelocity()
        {
            return comVelocity;
        }

        const Eigen::Vector3d& getCoMJdotQdot()
        {
            return comJdotQdot;
        }

        const Eigen::Matrix<double,3,Eigen::Dynamic>& getCoMJacobian()
        {
            return comJacobian;
        }

        const Eigen::Matrix<double,3,Eigen::Dynamic>& getCoMJacobianDot()
        {
            return comJacobianDot;
        }

        double getSegmentMass(int index)
        {
            return segMass[index];
        }

        const Eigen::Vector3d& getSegmentCoM(int index)
        {
            return segCoM[index];
        }

        const Eigen::Matrix<double,6,6>& getSegmentMassMatrix(int index)
        {
            return segMassMatrix[index];
        }

        const Eigen::Vector3d& getSegmentMomentsOfInertia(int index)
        {
            return segMomentsOfInertia[index];
        }

        const Eigen::Rotation3d& getSegmentInertiaAxes(int index)
        {
            return segInertiaAxes[index];
        }

        const Eigen::Displacementd& getSegmentPosition(int index)
        {
            segPosition[index] = KDLFrameToDisplacement(kdlmodel.getSegmentPosition(index));
            return segPosition[index];
        }

        const Eigen::Twistd& getSegmentVelocity(int index)
        {
            segVelocity[index] = KDLTwistToTwist(kdlmodel.getSegmentVelocity(index)).changeFrame(getSegmentPosition(index).getRotation().inverse());	
            return segVelocity[index];
        }

        const Eigen::Matrix<double,6,Eigen::Dynamic>& getSegmentJacobian(int index)
        {
            KDL::Jacobian kdljac(7);
            kdljac = kdlmodel.getSegmentJacobian(index);
			kdljac.changeBase(kdlmodel.getSegmentPosition(index).M.Inverse());
            segJacobian[index].block<3, 7>(0, 0) = kdljac.data.block<3, 7>(3, 0);
            segJacobian[index].block<3, 7>(3, 0) = kdljac.data.block<3, 7>(0, 0);
            return segJacobian[index];
        }

        const Eigen::Matrix<double,6,Eigen::Dynamic>& getSegmentJdot(int index)
        {
            return segJdot[index];
        }

        const Eigen::Matrix<double,6,Eigen::Dynamic>& getJointJacobian(int index)
        {
            segJointJacobian[index] = getSegmentJacobian(index);
            return segJointJacobian[index];
        }

        const Eigen::Twistd& getSegmentJdotQdot(int index)
        {
            return segJdotQdot[index];
        }

        void doSetJointPositions(const Eigen::VectorXd& _q)
        {
            q = _q;
            std::vector<double> stdq;
            for(int i=0; i<q.size();++i){
                stdq.push_back(q(i));
            }
            //(_q.data(), _q.data()+sizeof(_q.data())/sizeof(double));
            kdlmodel.setJointPosition(stdq);
        }

        void doSetJointVelocities(const Eigen::VectorXd& _dq)
        {
            dq    = _dq;
            alldq =  dq;
            std::vector<double> stddq;//(_dq.data(), _dq.data()+sizeof(_dq.data())/sizeof(double));
            for(int i=0; i<dq.size();++i){
                stddq.push_back(dq(i));
            }
            kdlmodel.setJointVelocity(stddq);
        }

        void doSetFreeFlyerPosition(const Eigen::Displacementd& _Hroot)
        {
            Hroot = _Hroot;
        }

        void doSetFreeFlyerVelocity(const Eigen::Twistd& _Troot)
        {
            Troot = _Troot;
        }

        int doGetSegmentIndex(const std::string& name)
        {
            try
            {
                return segIndexFromName.at(name);
            }
            catch(std::out_of_range)
            {
                std::stringstream ss;
                ss << "[orckukakdl::doGetSegmentIndex]: The input segment name '"+name+"' does not exist in this robot; possible key:\n";
                for (std::map< std::string, int >::iterator it = segIndexFromName.begin(); it != segIndexFromName.end(); ++it)
                    ss << it->first <<"\n";
                throw std::out_of_range(ss.str());
            }

        }

        const std::string& doGetSegmentName(int index)
        {
            return segNameFromIndex.at(index);
        }
};

OrcKukaKDL::OrcKukaKDL(const std::string& robotName) : 
    orc::Model(robotName, 7, false),
    pimpl(new Pimpl()){
}

OrcKukaKDL::~OrcKukaKDL(){

}

int OrcKukaKDL::nbSegments() const
{
    return pimpl->nbSegments();
}

const Eigen::VectorXd& OrcKukaKDL::getActuatedDofs() const
{
    return pimpl->getActuatedDofs();
}

const Eigen::VectorXd& OrcKukaKDL::getJointLowerLimits() const
{
    return pimpl->getJointLowerLimits();
}

const Eigen::VectorXd& OrcKukaKDL::getJointUpperLimits() const
{
    return pimpl->getJointUpperLimits();
}

const Eigen::VectorXd& OrcKukaKDL::getJointPositions() const
{
    return pimpl->getJointPositions();
}

const Eigen::VectorXd& OrcKukaKDL::getJointVelocities() const
{
    return pimpl->getJointVelocities();
}

const Eigen::Displacementd& OrcKukaKDL::getFreeFlyerPosition() const
{
    return pimpl->getFreeFlyerPosition();
}

const Eigen::Twistd& OrcKukaKDL::getFreeFlyerVelocity() const
{
    return pimpl->getFreeFlyerVelocity();
}

const Eigen::MatrixXd& OrcKukaKDL::getInertiaMatrix() const
{
    return pimpl->getInertiaMatrix();
}

const Eigen::MatrixXd& OrcKukaKDL::getInertiaMatrixInverse() const
{
    return pimpl->getInertiaMatrixInverse();
}

const Eigen::MatrixXd& OrcKukaKDL::getDampingMatrix() const
{
    return pimpl->getDampingMatrix();
}

const Eigen::VectorXd& OrcKukaKDL::getNonLinearTerms() const
{
    return pimpl->getNonLinearTerms();
}

const Eigen::VectorXd& OrcKukaKDL::getLinearTerms() const
{
    return pimpl->getLinearTerms();
}

const Eigen::VectorXd& OrcKukaKDL::getGravityTerms() const
{
    return pimpl->getGravityTerms();
}

double OrcKukaKDL::getMass() const
{
    return pimpl->getMass();
}

const Eigen::Vector3d& OrcKukaKDL::getCoMPosition() const
{
    return pimpl->getCoMPosition();
}

const Eigen::Vector3d& OrcKukaKDL::getCoMVelocity() const
{
    return pimpl->getCoMVelocity();
}

const Eigen::Vector3d& OrcKukaKDL::getCoMJdotQdot() const
{
    return pimpl->getCoMJdotQdot();
}

const Eigen::Matrix<double,3,Eigen::Dynamic>& OrcKukaKDL::getCoMJacobian() const
{
    return pimpl->getCoMJacobian();
}

const Eigen::Matrix<double,3,Eigen::Dynamic>& OrcKukaKDL::getCoMJacobianDot() const
{
    return pimpl->getCoMJacobianDot();
}

double OrcKukaKDL::getSegmentMass(int index) const
{
    return pimpl->getSegmentMass(index);
}

const Eigen::Vector3d& OrcKukaKDL::getSegmentCoM(int index) const
{
    return pimpl->getSegmentCoM(index);
}

const Eigen::Matrix<double,6,6>& OrcKukaKDL::getSegmentMassMatrix(int index) const
{
    return pimpl->getSegmentMassMatrix(index);
}

const Eigen::Vector3d& OrcKukaKDL::getSegmentMomentsOfInertia(int index) const
{
    return pimpl->getSegmentMomentsOfInertia(index);
}

const Eigen::Rotation3d& OrcKukaKDL::getSegmentInertiaAxes(int index) const
{
    return pimpl->getSegmentInertiaAxes(index);
}

const Eigen::Displacementd& OrcKukaKDL::getSegmentPosition(int index) const
{
    return pimpl->getSegmentPosition(index);
}

const Eigen::Twistd& OrcKukaKDL::getSegmentVelocity(int index) const
{
    return pimpl->getSegmentVelocity(index);
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& OrcKukaKDL::getSegmentJacobian(int index) const
{
    return pimpl->getSegmentJacobian(index);
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& OrcKukaKDL::getSegmentJdot(int index) const
{
    return pimpl->getSegmentJdot(index);
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& OrcKukaKDL::getJointJacobian(int index) const
{
    return pimpl->getJointJacobian(index);
}

const Eigen::Twistd& OrcKukaKDL::getSegmentJdotQdot(int index) const
{
    return pimpl->getSegmentJdotQdot(index);
}

void OrcKukaKDL::doSetJointPositions(const Eigen::VectorXd& q)
{
    pimpl->doSetJointPositions(q);
}

void OrcKukaKDL::doSetJointVelocities(const Eigen::VectorXd& dq)
{
    pimpl->doSetJointVelocities(dq);
}

void OrcKukaKDL::doSetFreeFlyerPosition(const Eigen::Displacementd& Hroot)
{
    pimpl->doSetFreeFlyerPosition(Hroot);
}

void OrcKukaKDL::doSetFreeFlyerVelocity(const Eigen::Twistd& Troot)
{
    pimpl->doSetFreeFlyerVelocity(Troot);
}

int OrcKukaKDL::doGetSegmentIndex(const std::string& name) const
{
    return pimpl->doGetSegmentIndex(name);
}

const std::string& OrcKukaKDL::doGetSegmentName(int index) const
{
    return pimpl->doGetSegmentName(index);
}

void OrcKukaKDL::printAllData() const{
    return;
}

