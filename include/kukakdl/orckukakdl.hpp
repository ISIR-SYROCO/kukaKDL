// Filename:  orckukakdl.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#ifndef ORCKUKAKDL_HPP
#define ORCKUKAKDL_HPP 

#include "kukakdl/kukakdl.hpp"
#include "orc/control/Model.h"
#include <boost/smart_ptr.hpp>

class OrcKukaKDL : public orc::Model{
    public:
        //================ General Methods =================//
        virtual int                          nbSegments               () const;
        virtual const Eigen::VectorXd&       getActuatedDofs          () const;
        virtual const Eigen::VectorXd&       getJointLowerLimits      () const;
        virtual const Eigen::VectorXd&       getJointUpperLimits      () const;
        virtual const Eigen::VectorXd&       getJointPositions        () const;
        virtual const Eigen::VectorXd&       getJointVelocities       () const;
        virtual const Eigen::Displacementd&  getFreeFlyerPosition     () const;
        virtual const Eigen::Twistd&         getFreeFlyerVelocity     () const;

        //================ Dynamic Methods =================//
        virtual const Eigen::MatrixXd&       getInertiaMatrix         () const;
        virtual const Eigen::MatrixXd&       getInertiaMatrixInverse  () const;
        virtual const Eigen::MatrixXd&       getDampingMatrix         () const;
        virtual const Eigen::VectorXd&       getNonLinearTerms        () const;
        virtual const Eigen::VectorXd&       getLinearTerms           () const;
        virtual const Eigen::VectorXd&       getGravityTerms          () const;

        //================== CoM Methods ===================//
        virtual double                                         getMass                     () const;
        virtual const Eigen::Vector3d&                         getCoMPosition              () const;
        virtual const Eigen::Vector3d&                         getCoMVelocity              () const;
        virtual const Eigen::Vector3d&                         getCoMJdotQdot              () const;
        virtual const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMJacobian              () const;
        virtual const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMJacobianDot           () const;

        //================ Segments Methods ================//
        virtual double                                         getSegmentMass              (int index) const;
        virtual const Eigen::Vector3d&                         getSegmentCoM               (int index) const;
        virtual const Eigen::Matrix<double,6,6>&               getSegmentMassMatrix        (int index) const;
        virtual const Eigen::Vector3d&                         getSegmentMomentsOfInertia  (int index) const;
        virtual const Eigen::Rotation3d&                       getSegmentInertiaAxes       (int index) const;
        virtual const Eigen::Displacementd&                    getSegmentPosition          (int index) const;
        virtual const Eigen::Twistd&                           getSegmentVelocity          (int index) const;
        virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&  getSegmentJacobian          (int index) const;
        virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&  getSegmentJdot              (int index) const;
        virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&  getJointJacobian            (int index) const;
        virtual const Eigen::Twistd&                           getSegmentJdotQdot          (int index) const;

        //================= Other Methods ==================//
        virtual void printAllData() const;

        OrcKukaKDL(const std::string& robotName);
        virtual ~OrcKukaKDL();

    protected:
        //=============== Set State Methods ================//
        virtual void                doSetJointPositions     (const Eigen::VectorXd& q);
        virtual void                doSetJointVelocities    (const Eigen::VectorXd& dq);
        virtual void                doSetFreeFlyerPosition  (const Eigen::Displacementd& Hroot);
        virtual void                doSetFreeFlyerVelocity  (const Eigen::Twistd& Troot);

        //=============== Index Name Methods ===============//
        virtual int                 doGetSegmentIndex       (const std::string& name) const;
        virtual const std::string&  doGetSegmentName        (int index) const;

    private:
        struct Pimpl;
        boost::shared_ptr< Pimpl > pimpl;

};

extern "C"
{
    orc::Model* Create_kukakdl(const std::string& robotName)
    {
        return new OrcKukaKDL(robotName);
    }

}  

#endif /* ORCKUKAKDL_HPP */
