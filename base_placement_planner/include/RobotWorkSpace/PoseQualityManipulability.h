/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once
#include "PoseQualityMeasurement.h"
#include "RobotWorkSpace.h"

namespace RobotWorkSpace
{
    /**
    *
    * \class PoseQualityManipulability
    *
    * This measurement computes the Yoshikawa's manipulability index by computing the Singular value Decomposition (SVD) of the Jacobian.
    * Two modes are offered: Either all singular values are multiplied
    * or the ratio of minimum and maximum singular value is returned (also known as (inverted) Condition number).
    */
    class PoseQualityManipulability :  public RobotWorkSpace::PoseQualityMeasurement
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        enum ManipulabilityIndexType
        {
            eMultiplySV,    // multiply all singular values
            eMinMaxRatio    // ratio of max and min singular value (aka (inverted) Condition number)  condition Number
        };

        PoseQualityManipulability(RobotWorkSpace::YAMLConfig robotConfig, KDL::JntArray nominal, ManipulabilityIndexType i = eMinMaxRatio);
        ~PoseQualityManipulability();

        /*!
            Returns the manipulability of the current configuration.
        */
        float getPoseQuality() override;
        virtual float getManipulability(ManipulabilityIndexType i);

        /*!
            카테시안 방향에 따른 Manipulability
            The quality is determined for a given Cartesian direction.
            The current configuration of the corresponding RNS is analyzed and the quality is returned.
            See derived classes for details.
            \param direction A 3d or 6d vector with the Cartesian direction to investigate.
        */
        float getPoseQuality(const Eigen::VectorXf& direction) override;
        virtual float getManipulability(const Eigen::VectorXf& direction, int considerFirstSV = -1);

        /*!
            Considers the first considerFirstSV singular values and ignores any remaining entries in the sv vector.
            This can be useful, when constrained motions should be analyzed (e.g. 2d motions in 3d)
        */
        virtual float getManipulability(ManipulabilityIndexType i, int considerFirstSV);

        /*!
            Returns the singular values of the current Jacobian. The values are ordered, starting with the largest value.
        */
        Eigen::VectorXf getSingularValues();

        /*
            Computes the scaled singular vectors (U) of the Jacobian with SVD.
            The columns of the returned matrix are the Cartesian singular vectors of the Jacobi, scaled with 1/maxSingularValue.
            The columns of U are called 'Cartesian Vectors' since they describe the Cartesian relationship.
            SVD:  J = USV'
        */
        Eigen::MatrixXf getSingularVectorCartesian();

        /*
            Enable or disable joint limit considerations.
            If enabled, joint limits are considered by applying this penalize function:
            P(\theta_i) = 1 - \exp (-k \prod_{i=1}^n { \frac{(\theta_i - l_i^-)(l_i^+ - \theta_i)} { (l_i^+ - l_i^-)^2 } } ).

            The joint limits are given by l_i^- and l_i^+  and k is a factor that can be used to adjust the behavior
        */
        virtual void penalizeJointLimits(bool enable, float k = 50.0f);

        bool consideringJointLimits() override;

        static std::string getTypeName();

        PoseQualityMeasurementPtr clone() override;

    protected:
        float getJointLimitPenalizationFactor();

        Eigen::MatrixXf jacobian;
        ManipulabilityIndexType manipulabilityType;

        bool penJointLimits;
        float penJointLimits_k;

        float penalizeRotationFactor; // to align translational and rotational components

        //! if set, the Jacobian is computed in [m], while assuming the kinematic definitions to be in [mm]
        bool convertMMtoM;
    };
}

