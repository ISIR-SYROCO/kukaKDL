// Filename:  orckukakdl.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#ifndef ORCKUKAKDL_HPP
#define ORCKUKAKDL_HPP 

#include "kukakdl/kukakdl.hpp"
#include "orc/control/Model.h"

class OrcKukaKDL : public orc::Model{
    public:
        //================ General Methods =================//
        virtual int                          nbSegments               ();
        virtual Eigen::VectorXd&       getActuatedDofs          ();
        virtual Eigen::VectorXd&       getJointLowerLimits      ();
        virtual Eigen::VectorXd&       getJointUpperLimits      ();
        virtual Eigen::VectorXd&       getJointPositions        ();
        virtual Eigen::VectorXd&       getJointVelocities       ();
        virtual Eigen::Displacementd&  getFreeFlyerPosition     ();
        virtual Eigen::Twistd&         getFreeFlyerVelocity     ();

        //================ Dynamic Methods =================//
        virtual Eigen::MatrixXd&       getInertiaMatrix         ();
        virtual Eigen::MatrixXd       getInertiaMatrixInverse  ();
        virtual const Eigen::MatrixXd&       getDampingMatrix         () const;
        virtual Eigen::VectorXd&       getNonLinearTerms        ();
        virtual const Eigen::VectorXd&       getLinearTerms           () const;
        virtual Eigen::VectorXd&       getGravityTerms          ();

        //================== CoM Methods ===================//
        virtual double                                         getMass                     () const;
        virtual const Eigen::Vector3d&                         getCoMPosition              () const;
        virtual const Eigen::Vector3d&                         getCoMVelocity              () const;
        virtual const Eigen::Vector3d&                         getCoMJdotQdot              () const;
        virtual const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMJacobian              () const;
        virtual const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMJacobianDot           () const;

        //================ Segments Methods ================//
        virtual double                                         getSegmentMass              (int index);
        virtual const Eigen::Vector3d&                         getSegmentCoM               (int index) const;
        virtual const Eigen::Matrix<double,6,6>&               getSegmentMassMatrix        (int index) const;
        virtual const Eigen::Vector3d&                         getSegmentMomentsOfInertia  (int index) const;
        virtual const Eigen::Rotation3d&                       getSegmentInertiaAxes       (int index) const;
        virtual Eigen::Displacementd                    getSegmentPosition          (int index);
        virtual Eigen::Twistd                           getSegmentVelocity          (int index);
        virtual Eigen::Matrix<double,6,Eigen::Dynamic>  getSegmentJacobian          (int index);
        virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&  getSegmentJdot              (int index) const;
        virtual Eigen::Matrix<double,6,Eigen::Dynamic>  getJointJacobian            (int index);
        virtual const Eigen::Twistd&                           getSegmentJdotQdot          (int index) const;

        //================= Other Methods ==================//
        virtual void printAllData() const;

        KukaKDL kdlmodel;
        Eigen::Displacementd H_root;
        Eigen::Twistd T_root;
        Eigen::MatrixXd B;
        Eigen::VectorXd l;

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

};

#endif /* ORCKUKAKDL_HPP */
