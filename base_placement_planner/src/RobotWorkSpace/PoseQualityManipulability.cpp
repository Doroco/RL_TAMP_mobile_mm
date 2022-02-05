#include "../include/RobotWorkSpace/PoseQualityManipulability.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;


namespace RobotWorkSpace
{
    PoseQualityManipulability::PoseQualityManipulability(RobotWorkSpace::YAMLConfig robotConfig, KDL::JntArray nominal, ManipulabilityIndexType i)
        : PoseQualityMeasurement(robotConfig,nominal), manipulabilityType(i), penJointLimits(true), convertMMtoM(false)
    {
        name = getTypeName();
        jacobian.resize(6,nb_of_joints_);
        jacobian.setZero();

        // Jacobian 회전에 어떤 penalize를 주는 지 파악하고 사용할 것!!!!!!!! 0.15
        penalizeRotationFactor = 1.0f;; // translation<->rotation factors   1.0f;
    }


    PoseQualityManipulability::~PoseQualityManipulability()
    = default;


    Eigen::MatrixXf PoseQualityManipulability::getSingularVectorCartesian()
    {
        if (verbose)
        {
            std::cout << "*** PoseQualityManipulability::getSingularVectorCartesian()\n";
        }


        Eigen::MatrixXf jac = getJacobian();
        //cout << "JAC\n:" << jac << std::endl;
        // penalize rotation
        jac.block(3, 0, 3, jac.cols()) *= penalizeRotationFactor;
        //cout << "scaled JAC\n:" << jac << std::endl;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXf U = svd.matrixU();
        Eigen::VectorXf sv = svd.singularValues();

        if (verbose)
        {
            std::cout << "U:\n" << U << std::endl;
        }

        if (sv.rows() == 0)
        {
            return U;
        }

        if (verbose)
        {
            std::cout << "sv:\n" << sv << std::endl;
        }

        float maxEV = sv(0);

        if (verbose)
        {
            std::cout << "maxEV:" << maxEV << std::endl;
        }

        /*for (int i=0;i<sv.rows();i++)
        {
            Eigen::MatrixXf Bl = U.block(0,i,U.rows(),1);
            Bl *= sv(i) / maxEV;
            U.block(0,i,U.rows(),1) = Bl;
        }*/
        if (verbose)
        {
            std::cout << "result:\n" << U << std::endl;
        }

        return U;
    }



    float PoseQualityManipulability::getManipulability(ManipulabilityIndexType i)
    {
        return getManipulability(i, -1);
    }

    float PoseQualityManipulability::getManipulability(const Eigen::VectorXf& direction, int /*considerFirstSV*/)
    {
        assert(direction.rows() == 3 || direction.rows() == 6);
        Eigen::VectorXf d(6);

        if (direction.rows() == 6)
        {
            d = direction;
        }
        else
        {
            d.setZero();
            d.segment(0, 3) = direction;
        }

        float result = 0;

        // jacobian (in global coord system!)
        Eigen::MatrixXf jac = getJacobianGlobal();
        // penalize rotation
        jac.block(3, 0, 3, jac.cols()) *= penalizeRotationFactor;

        // compute gradient
        Eigen::VectorXf gradient = (jac.transpose() * d).transpose();

        result = gradient.norm();


        if (penJointLimits)
        {
            result *= getJointLimitPenalizationFactor();
        }

        return result;

    }

    float PoseQualityManipulability::getManipulability(ManipulabilityIndexType i, int considerFirstSV)
    {
        Eigen::VectorXf sv = getSingularValues();

        if (considerFirstSV <= 0 || considerFirstSV > sv.rows())
        {
            considerFirstSV = sv.rows();
        }

        float result = 0.0f;

        switch (i)
        {
            case eMultiplySV:
            {
                if (sv.rows() >= 1)
                {
                    result = sv(0);

                    for (int j = 1; j < considerFirstSV; j++)
                    {
                        result *= sv(j);
                    }
                }

                break;
            }

            case eMinMaxRatio:
            {
                if (sv.rows() >= 2)
                {
                    float maxSV = sv(0);
                    float minSV = sv(considerFirstSV - 1);

                    if (maxSV != 0)
                    {
                        result = minSV / maxSV;
                    }
                }

                break;
            }

            default:
                std::cout << "Manipulability type not implemented..." << std::endl;
        }

        if (penJointLimits)
        {
            result *= getJointLimitPenalizationFactor();
        }

        return result;
    }

    float PoseQualityManipulability::getPoseQuality()
    {
        return getManipulability(manipulabilityType);
    }

    float PoseQualityManipulability::getPoseQuality(const Eigen::VectorXf& direction)
    {
        return getManipulability(direction);
    }

    Eigen::VectorXf PoseQualityManipulability::getSingularValues()
    {
        Eigen::MatrixXf jac = getJacobian();
        // penalize rotation
        jac.block(3, 0, 3, jac.cols()) *= penalizeRotationFactor;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
        return svd.singularValues();
    }

    void PoseQualityManipulability::penalizeJointLimits(bool enable, float k /*= 50.0f*/)
    {
        penJointLimits = enable;
        penJointLimits_k = k;
    }

    float PoseQualityManipulability::getJointLimitPenalizationFactor()
    {
        float p = 1.0f;

        for (unsigned int i = 0; i < nb_of_joints_; i++)
        {
            float d = joint_limit_.upper_rad_(i) - joint_limit_.lower_rad_(i);
            d = d * d;
            float a = (nominal_(i) - joint_limit_.lower_rad_(i)) * (joint_limit_.upper_rad_(i) - nominal_(i));

            if (d != 0)
            {
                p *= a / d;
            }
        }

        float result = 1.0f - exp(-penJointLimits_k * p);
        if (verbose)
            std::cout << "Pen factor:" << result << std::endl;
        return result;
    }

    std::string PoseQualityManipulability::getTypeName()
    {
        return std::string("PoseQualityManipulability");
    }

    bool PoseQualityManipulability::consideringJointLimits()
    {
        return penJointLimits;
    }

    PoseQualityMeasurementPtr PoseQualityManipulability::clone()
    {
        PoseQualityManipulabilityPtr m(new PoseQualityManipulability(this->config_,this->nominal_, this->manipulabilityType));
        m->penalizeJointLimits(this->penJointLimits, this->penJointLimits_k);
        return m;
    }


}
