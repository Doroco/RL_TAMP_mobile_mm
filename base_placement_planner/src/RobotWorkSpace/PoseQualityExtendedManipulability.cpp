#include "../include/RobotWorkSpace/PoseQualityExtendedManipulability.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cfloat>



using namespace std;

namespace RobotWorkSpace
{


    PoseQualityExtendedManipulability::PoseQualityExtendedManipulability(RobotWorkSpace::YAMLConfig robotConfig, KDL::JntArray nominal, PoseQualityManipulability::ManipulabilityIndexType i)
        : PoseQualityManipulability(robotConfig, nominal, i)
    {
        name = getTypeName();
        createCartDimPermutations(cartDimPermutations);
        considerObstacle = false;
        obstacleDir.setZero();
        obstacle_alpha = 1.0f;
        obstacle_beta = 1.0f;
        tmpJac.resize(6, nb_of_joints_);
    }

    PoseQualityExtendedManipulability::~PoseQualityExtendedManipulability()
    = default;

    float PoseQualityExtendedManipulability::getPoseQuality()
    {
        return getPoseQuality(manipulabilityType, -1);
    }

    float PoseQualityExtendedManipulability::getPoseQuality(PoseQualityManipulability::ManipulabilityIndexType i, int considerFirstSV)
    {
        jacobian = getJacobian();
        return getPoseQuality(jacobian, i, considerFirstSV);
    }

    float PoseQualityExtendedManipulability::getPoseQuality(Eigen::MatrixXf jac, PoseQualityManipulability::ManipulabilityIndexType i, int considerFirstSV)
    {
        //extManipData d;
        currentManipData.reset();

        if (!getDetailedAnalysis(jac, currentManipData, considerFirstSV))
        {
            std::cout << "ERROR" << std::endl;
            return 0;
        }

        switch (i)
        {
            case eMultiplySV:
            {
                return currentManipData.extManip_Volume;
            }
            break;

            case eMinMaxRatio:
            {
                return currentManipData.extManip_InvCondNumber;
            }
            break;

            default:
                std::cout << "NYI..." << std::endl;
        }

        return 0;
    }

    float PoseQualityExtendedManipulability::getPoseQuality(const Eigen::VectorXf& direction)
    {
        return getManipulability(direction);
    }


    void PoseQualityExtendedManipulability::getQualityWeighting(float jv, float limitMin, float limitMax, float& storeWeightMin, float& storeWeightMax)
    {
        float w;

        if (fabs(limitMax - jv) > 1e-10 && fabs(jv - limitMin) > 1e-10)
        {
            w = ((limitMax - limitMin) * (limitMax - limitMin) * (2.0f * jv - limitMax - limitMin)) / (4.0f * (limitMax - jv) * (limitMax - jv) * (jv - limitMin) * (jv - limitMin));
        }
        else
        {
            w = 1e10;
        }

        // w is \delta H / \delta q, hence we are only interested in the direction -> fabs, a neg value wouldn't make sense in further processing
        w = fabs(w);

        if (std::isinf(w) || std::isinf(-w) || std::isnan(w))
        {
            std::cout << "nan" << std::endl;
        }


        if (fabs(jv - limitMin) > fabs(jv - limitMax))
        {
            // next to upper limit
            storeWeightMin = 1.0f;
            storeWeightMax = 1.0f + w;
        }
        else
        {
            // next to lower limit
            storeWeightMin = 1.0f + w;
            storeWeightMax = 1.0f;
        }
    }

    void PoseQualityExtendedManipulability::getPenalizations(Eigen::VectorXf& penLo, Eigen::VectorXf& penHi)
    {
        penLo.resize(nb_of_joints_);
        penHi.resize(nb_of_joints_);

        for (size_t i = 0; i < nb_of_joints_; i++)
        {
            float l, h;
            getQualityWeighting(nominal_(i), joint_limit_.lower_rad_(i), joint_limit_.upper_rad_(i), l, h);
            penLo(i) = 1.0f / sqrtf(l);
            penHi(i) = 1.0f / sqrtf(h);
        }

        if (verbose)
        {
            std::cout << "PEN LO:";
            //VirtualRobot::MathTools::print(penLo);
            std::cout << "PEN HI:";
            //VirtualRobot::MathTools::print(penHi);
        }
    }

    void PoseQualityExtendedManipulability::getObstaclePenalizations(const Eigen::Vector3f& obstVect, const Eigen::MatrixXf& jac, Eigen::MatrixXf& penObstLo, Eigen::MatrixXf& penObstHi)
    {
        float scaleFactor = 0.001f; 

        penObstLo.resize(jac.rows(), jac.cols());
        penObstHi.resize(jac.rows(), jac.cols());
        penObstLo.setZero();
        penObstHi.setZero();

        assert(jac.rows() == 6);

        float d = obstVect.norm();

        if (d < 1e-10)
        {
            return;
        }

        d *= scaleFactor; // m로 단위변환

        // compute \delta P / \delta d
        float p1 = (float) - (exp((double)(-obstacle_alpha * d)) * pow((double)d, (double)(-obstacle_beta)) * (double)(obstacle_beta * 1.0f / d + obstacle_alpha));

        if (verbose)
        {
            std::cout << "++++++++++++++++++++++++++++++" << std::endl;
            std::cout << "+++++++++++++++++++ d:" << d << std::endl;
            std::cout << "+++++++++++++++++++ p1:" << p1 << std::endl;
            std::cout << "++++++++++++++++++++++++++++++" << std::endl;
        }

        // 임시로 obstVect 설정하기
        Eigen::VectorXf v2(6);
        v2.setZero();
        v2.block(0, 0, 3, 1) = obstVect;
        v2 *= scaleFactor;

        // 벡터 노말리제이션!!
        Eigen::Vector3f obstDirNorm = obstVect;
        obstDirNorm.normalize();

        // compute gradient
        Eigen::VectorXf penObst = (jac.transpose() * v2).transpose();

        if (verbose)
        {
            std::cout << "PEN OBST (gradient)1:";
            //VirtualRobot::MathTools::print(penObst);
        }

        penObst = penObst / d;

        if (verbose)
        {

            std::cout << "PEN OBST (gradient)2:";
            //VirtualRobot::MathTools::print(penObst);
        }

        penObst *= p1;

        if (verbose)
        {
            std::cout << "PEN OBST (gradient):";
            //VirtualRobot::MathTools::print(penObst);
        }

        // compute pen factor from gradient
        for (size_t j = 0; j < nb_of_joints_; j++)
        {
            penObst(j) = 1.0f / sqrtf(1.0f + fabs(penObst(j)));
        }

        if (verbose)
        {
            std::cout << "PEN OBST (pen factor):";
            //VirtualRobot::MathTools::print(penObst);
        }

        // 그냥 싹다 1로 만드는 거 같음
        penObstLo.setConstant(1.0f);
        penObstHi.setConstant(1.0f);

        for (int i = 0; i < 3; i++)
        {
            //fabs(obstDirNorm(i))*distQual + (1.0f - fabs(obstDirNorm(i)))*1.0f
            Eigen::VectorXf scPen = penObst;

            //for (int j=0;j<penObst.rows();j++)
            //  scPen(j) = fabs(obstDirNorm(i))*scPen(j) + (1.0f - fabs(obstDirNorm(i)))*1.0f;
            if (obstVect(i) > 0)
            {
                penObstHi.block(i, 0, 1, penObstHi.cols()) = scPen.transpose();
            }
            else
            {
                penObstLo.block(i, 0, 1, penObstLo.cols()) = scPen.transpose();
            }
        }

        if (verbose)
        {

            std::cout << "PEN OBST (penObstLo):\n" << penObstLo << std::endl;
            std::cout << "PEN OBST (penObstHi):\n" << penObstHi << std::endl;
        }

        if (verbose)
        {

            std::cout << "ObstVect: d=" << obstVect.norm() << ", v=";
            //MathTools::print(obstVect);
        }
    }

    Eigen::MatrixXf PoseQualityExtendedManipulability::getJacobianWeightedObstacles(const Eigen::MatrixXf& jac, const std::vector<float>& directionVect, const Eigen::VectorXf& penLo, const Eigen::VectorXf& penHi, const Eigen::MatrixXf& penObstLo, const Eigen::MatrixXf& penObstHi)
    {
        Eigen::MatrixXf res = jac;

        assert(penHi.rows() == penLo.rows());
        assert(directionVect.size() == 6);

        for (int i = 0; i < jac.rows(); i++) // cart
        {
            for (int j = 0; j < jac.cols(); j++) // joints
            {
                bool posPen = true; // jl

                // check for neg quadrant
                if (directionVect[i] < 0)
                {
                    posPen = !posPen;
                    res(i, j) *= penObstLo(i, j);
                }
                else
                {
                    res(i, j) *= penObstHi(i, j);
                }

                // check for inverted movement of joint
                if (jac(i, j) < 0)
                {
                    posPen = !posPen;
                }

                if (posPen)
                {
                    res(i, j) *= penHi(j);
                }
                else
                {
                    res(i, j) *= penLo(j);
                }
            }
        }

        return res;
    }


    Eigen::MatrixXf PoseQualityExtendedManipulability::getJacobianWeighted(const Eigen::MatrixXf& jac, const std::vector<float>& directionVect, const Eigen::VectorXf& penLo, const Eigen::VectorXf& penHi)
    {
        Eigen::MatrixXf res = jac;

        assert(penHi.rows() == penLo.rows());
        assert(directionVect.size() == 6);

        //Columns of the Jacobian matrix are associated with joints of the robot. 
        //Each column in the Jacobian matrix represents the effect on end-effector velocities due to variation in each joint velocity.

        for (int i = 0; i < jac.rows(); i++) // cart
        {
            for (int j = 0; j < jac.cols(); j++) // joints
            {
                bool posPen = true;

                // check for neg quadrant
                if (directionVect[i % 6] < 0)
                {
                    posPen = !posPen;
                }

                // check for inverted movement of joint  openchain의 경우이구나
                if (jac(i, j) < 0)
                {
                    posPen = !posPen;
                }

                if (posPen)
                {
                    res(i, j) *= penHi(j);
                }
                else
                {
                    res(i, j) *= penLo(j);
                }
            }
        }

        return res;
    }

    bool PoseQualityExtendedManipulability::analyzeJacobian(const Eigen::MatrixXf& jac, Eigen::VectorXf& sv, Eigen::MatrixXf& singVectors, Eigen::MatrixXf& U, Eigen::MatrixXf& V, bool printInfo)
    {
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
        U = svd.matrixU();
        V = svd.matrixV();
        sv = svd.singularValues();

        //float maxEV = sv(0);

        // scale Cartesian SingularValues to scaled influence
        singVectors = U;

        for (int i = 0; i < sv.rows(); i++)
        {
            /*Eigen::MatrixXf Bl = U.block(0, i, U.rows(), 1); // cart sing vectors are normed (norm == 1)
            Bl *= sv(i);// / maxEV;
            singVectors.block(0, i, U.rows(), 1) = Bl;*/

            singVectors.block(0, i, U.rows(), 1) = U.block(0, i, U.rows(), 1) * sv(i);
        }

        if (printInfo)
        {
            std::cout << "Sing Values:\n" << sv << std::endl;
            std::cout << "U:\n" << U << std::endl;
            std::cout << "Sing Vectors (scaled according to SingValues:\n" << singVectors << std::endl;
        }

        return true;
    }

    bool PoseQualityExtendedManipulability::getDetailedAnalysis(extManipData& storeData, bool (&dims)[6], int considerFirstSV)
    {
        return getDetailedAnalysis(jacobian, storeData, dims, considerFirstSV);
    }

    bool PoseQualityExtendedManipulability::getDetailedAnalysis(const Eigen::MatrixXf& jac, extManipData& storeData, bool (&dims)[6], int considerFirstSV)
    {
        storeData.jac = jac;
        storeData.nrJoints = storeData.jac.cols();

        // penalize rotation
        storeData.jac.block(3, 0, 3, storeData.jac.cols()) *= penalizeRotationFactor;

        getPenalizations(storeData.penLo, storeData.penHi);
        storeData.obstacles = considerObstacle;

        if (considerObstacle)
        {
            getObstaclePenalizations(obstacleDir, storeData.jac, storeData.penObstLo, storeData.penObstHi);
        }

        //bool verbose = false;
        if (verbose)
        {
            std::cout << "considerFirstSV=" << considerFirstSV << std::endl;
            std::cout << "JAC:\n" << std::endl;
            ///MathTools::printMat(storeData.jac);
        }

        for (int i = 0; i < 64; i++)
        {
            if (considerObstacle)
            {
                storeData.jacPen[i] = getJacobianWeightedObstacles(storeData.jac, cartDimPermutations[i], storeData.penLo, storeData.penHi, storeData.penObstLo, storeData.penObstHi);
            }
            else
            {
                // 각 hypertoctant 사분면 마다 페널라이즈 텀이랑 자코비안을 결정해준다.
                storeData.jacPen[i] = getJacobianWeighted(storeData.jac, cartDimPermutations[i], storeData.penLo, storeData.penHi);
            }

            for (int j = 0; j < 6; j++)
            {
                // 디멘션이 설정이 안되있으면 패널라이즈 자코비안들을 0으로 만들어버린다.
                if (!dims[j])
                {
                    storeData.jacPen[i].block(j, 0, 1, storeData.jacPen[i].cols()).setConstant(0);
                }
            }

            if (verbose && i < 4)
            {
                std::cout << "JAC PEN:" << i << std::endl;
                std::cout<<storeData.jacPen[i]<<std::endl;
            }

            // SVD를 통해서 특이값 분해를 실행하여 저장한다.
            analyzeJacobian(storeData.jacPen[i], storeData.sv[i], storeData.singVectors[i], storeData.U[i], storeData.V[i], (verbose && i < 4));
        }

        // considerFirstSV 대충6이라고 생각하시오
        if (considerFirstSV <= 0 || considerFirstSV > storeData.sv[0].rows())
        {
            considerFirstSV = storeData.sv[0].rows();
        }

        storeData.consideFirstSV = considerFirstSV;
        float result_v = FLT_MAX;
        float result_c = FLT_MAX;
        float minSV = FLT_MAX;
        float maxSV = 0.0f;

        // 사분면을 모두 돌아다닌다.
        for (int k = 0; k < 64; k++)
        {
            Eigen::VectorXf sv = storeData.sv[k];//.block(0,k,svAll.rows(),1);
            //cout << "k: sv:";
            //VirtualRobot::MathTools::print(sv);
            float tmpRes = 0;

            // volume
            if (sv.rows() >= 1)
            {
                tmpRes = sv(0) * sv(0);

                for (int j = 1; j < considerFirstSV; j++)
                {
                    tmpRes *= sv(j) * sv(j);
                }

                tmpRes = sqrtf(tmpRes);
            }

            if (tmpRes < result_v)
            {
                result_v = tmpRes;
            }

            //cout << "## k:" << k << " -> tmpRes: " << tmpRes << ", result:" << result << std::endl;

            // cond numb
            if (sv.rows() >= 2)
            {
                float minSV_loc = FLT_MAX;
                float maxSV_loc = 0.0f;

                for (int j = 0; j < considerFirstSV; j++)
                {
                    if (sv(j) < minSV)
                    {
                        minSV = sv(j);
                    }

                    if (sv(j) > maxSV)
                    {
                        maxSV = sv(j);
                    }

                    if (sv(j) < minSV_loc)
                    {
                        minSV_loc = sv(j);
                    }

                    if (sv(j) > maxSV_loc)
                    {
                        maxSV_loc = sv(j);
                    }
                }

                if (maxSV_loc != 0)
                {
                    tmpRes = minSV_loc / maxSV_loc;
                }
                else
                {
                    tmpRes = 0;
                }

                if (tmpRes < result_c)
                {
                    result_c = tmpRes;

                    if (verbose)
                    {
                        std::cout << "## k:" << k << " -> minSV: " << minSV_loc << ", maxSV:" << maxSV_loc << std::endl;
                        std::cout << "## pen jac:\n" << std::endl;
                        std::cout<<storeData.jacPen[k]<<std::endl;
                    }
                }
            }

        }

        storeData.extManip_InvCondNumber = result_c;
        storeData.extManip_Volume = result_v;

        storeData.minSV = minSV;
        storeData.maxSV = maxSV;

        return true;
    }

    bool PoseQualityExtendedManipulability::getDetailedAnalysis(extManipData& storeData, int considerFirstSV)
    {
        if (considerFirstSV <= 0 || considerFirstSV > 6)
        {
            considerFirstSV = 6;
        }

        bool dims[6];
        int i = 0;

        for (i = 0; i < 6; i++)
        {
            dims[i] = true;
        }

        return getDetailedAnalysis(jacobian, storeData, dims, considerFirstSV);
    }

    bool PoseQualityExtendedManipulability::getDetailedAnalysis(Eigen::MatrixXf jac, extManipData& storeData, int considerFirstSV)
    {
        if (considerFirstSV <= 0 || considerFirstSV > 6)
        {
            considerFirstSV = 6;
        }

        bool dims[6];
        int i = 0;

        for (i = 0; i < 6; i++)
        {
            dims[i] = true;
        }

        return getDetailedAnalysis(jac, storeData, dims, considerFirstSV);
    }

    bool PoseQualityExtendedManipulability::createCartDimPermutations(std::vector < std::vector<float> >& storePerm)
    {
        for (int i = 0; i < 64; i++)
        {
            std::vector<float> dirVect;

            if (i % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            if ((i / 2) % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            if ((i / 4) % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            if ((i / 8) % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            if ((i / 16) % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            if ((i / 32) % 2 == 0)
            {
                dirVect.push_back(1);
            }
            else
            {
                dirVect.push_back(-1);
            }

            storePerm.push_back(dirVect);
        }

        return true;
    }

    std::vector < std::vector<float> > PoseQualityExtendedManipulability::getPermutationVector()
    {
        return cartDimPermutations;
    }

    void PoseQualityExtendedManipulability::considerObstacles(bool enable, float alpha /*= 1.0f*/, float beta /*= 1.0f*/)
    {
        PoseQualityManipulability::considerObstacle = enable;
        obstacle_alpha = alpha;
        obstacle_beta = beta;
    }

    std::string PoseQualityExtendedManipulability::getTypeName()
    {
        return std::string("PoseQualityExtendedManipulability_JLWeightsQuadrants");
    }

    bool PoseQualityExtendedManipulability::consideringJointLimits()
    {
        return true;
    }

    float PoseQualityExtendedManipulability::getManipulability(const Eigen::VectorXf& direction, int considerFirstSV)
    {
        ROS_ASSERT(direction.rows() == 3 || direction.rows() == 6);
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
        //VirtualRobot::DifferentialIKPtr jacobianGlobal(new VirtualRobot::DifferentialIK(rns));
        //jacobianGlobal->convertModelScalingtoM(convertMMtoM);
        Eigen::MatrixXf jacGlobal = getJacobian();//jacobianGlobal->getJacobianMatrix(rns->getTCP());
        // penalize rotation
        jacGlobal.block(3, 0, 3, jacGlobal.cols()) *= penalizeRotationFactor;


        Eigen::MatrixXf jacPen;
        // the joint limit penalizations in neg direction (n values)
        Eigen::VectorXf penLo;
        // the joint limit penalizations in pos direction (n values)
        Eigen::VectorXf penHi;
        // the obstacle penalizations in neg direction (6xn values)
        Eigen::MatrixXf penObstLo;
        // the joint limit penalizations in pos direction (6xn values)
        Eigen::MatrixXf penObstHi;

        std::vector<float> quadrant;

        //Find 6-DOF 사분면
        for (int i = 0; i < 6; i++)
            if (d(i) < 0)
            {
                quadrant.push_back(-1.0f);
            }
            else
            {
                quadrant.push_back(1.0f);
            }

        getPenalizations(penLo, penHi);

        if (considerObstacle)
        {
            Eigen::Vector3f zero = Eigen::Vector3f::Zero();
            Eigen::Vector3f pos = obstacleDir;
            //zero = rns->getTCP()->toGlobalCoordinateSystemVec(zero);
            //pos = rns->getTCP()->toGlobalCoordinateSystemVec(pos);

            // the orientation is skipped! (we have a position vector)
            Eigen::Vector3f obstVecGlobal;
            obstVecGlobal.block(0, 0, 3, 1) = pos - zero;
            getObstaclePenalizations(obstVecGlobal, jacGlobal, penObstLo, penObstHi);
            jacPen = getJacobianWeightedObstacles(jacGlobal, quadrant, penLo, penHi, penObstLo, penObstHi);
        }
        else
        {
            jacPen = getJacobianWeighted(jacGlobal, quadrant, penLo, penHi);
        }

        // compute gradient
        Eigen::VectorXf gradient = (jacPen.transpose() * d).transpose();

        // quality
        result = gradient.norm();


        // analyze corresponding hyperoctant
        Eigen::MatrixXf U;
        Eigen::MatrixXf V;
        Eigen::VectorXf sv;
        Eigen::MatrixXf singVectors;
        analyzeJacobian(jacPen, sv, singVectors, U, V, false);

        if (considerFirstSV <= 0 || considerFirstSV > sv.rows())
        {
            considerFirstSV = sv.rows();
        }

        float result_v = FLT_MAX;
        float result_c = FLT_MAX;
        float minSV = FLT_MAX;
        float maxSV = 0.0f;
        float tmpRes = 0;

        // volume
        if (sv.rows() >= 1)
        {
            tmpRes = sv(0) * sv(0);

            for (int j = 1; j < considerFirstSV; j++)
            {
                tmpRes *= sv(j) * sv(j);
            }

            tmpRes = sqrtf(tmpRes);
        }

        if (tmpRes < result_v)
        {
            result_v = tmpRes;
        }

        // cond numb
        if (sv.rows() >= 2)
        {
            float minSV_loc = FLT_MAX;
            float maxSV_loc = 0.0f;

            for (int j = 0; j < considerFirstSV; j++)
            {
                if (sv(j) < minSV)
                {
                    minSV = sv(j);
                }

                if (sv(j) > maxSV)
                {
                    maxSV = sv(j);
                }

                if (sv(j) < minSV_loc)
                {
                    minSV_loc = sv(j);
                }

                if (sv(j) > maxSV_loc)
                {
                    maxSV_loc = sv(j);
                }
            }

            if (maxSV_loc != 0)
            {
                tmpRes = minSV_loc / maxSV_loc;
            }
            else
            {
                tmpRes = 0;
            }

            if (tmpRes < result_c)
            {
                result_c = tmpRes;

                if (verbose)
                {
                    std::cout << "##  -> minSV: " << minSV_loc << ", maxSV:" << maxSV_loc << std::endl;
                    std::cout << "## pen jac:\n" << jacPen << std::endl;
                }
            }
        }

        return result * result_c;
    }

    PoseQualityMeasurementPtr PoseQualityExtendedManipulability::clone()
    {

        PoseQualityExtendedManipulabilityPtr m(new PoseQualityExtendedManipulability(this->config_,this->nominal_, this->manipulabilityType));
        m->penalizeJointLimits(this->penJointLimits, this->penJointLimits_k);
        m->considerObstacles(this->considerObstacle, this->obstacle_alpha, this->obstacle_beta);
        return m;
    }

    void PoseQualityExtendedManipulability::getSelfDistParameters(float &storeAlpha, float &storeBeta)
    {
        storeAlpha = obstacle_alpha;
        storeBeta = obstacle_beta;
    }
}
