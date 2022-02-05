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

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cfloat>
#include <memory>

namespace MathTools
{
    /************************************************************************/
    /* QUATERNIONS                                                          */
    /************************************************************************/

    struct Quaternion
    {
        Quaternion()
        {
            x = y = z = 0.0f;
            w = 1.0f;
        }
        float x, y, z, w;
    };


    /*!
        Return rotation that converts vector from to vector to.
    */
    Quaternion  getRotation(const Eigen::Vector3f& from, const Eigen::Vector3f& to);

    //! get the corresponding angle of rotation that is defined by the quaternion (radian)
    float  getAngle(const Quaternion& q);

    //! Return the quaternion that defines the difference between the two given rotations
    Quaternion  getDelta(const Quaternion& q1, const Quaternion& q2);

    //! Return the inverse quaternion
    Quaternion  getInverse(const Quaternion& q);

    //! Return the conjugated quaternion
    Quaternion  getConjugated(const Quaternion& q);

    //! Returns q1*q2
    Quaternion  multiplyQuaternions(const Quaternion& q1, const Quaternion& q2);

    //! returns q1 dot q2
    float  getDot(const Quaternion& q1, const Quaternion& q2);

    //! Computes mean orientation of quaternions
    MathTools::Quaternion  getMean(std::vector<MathTools::Quaternion> quaternions);

    /*!
    Apply the slerp interpolation
    \param q1 The first quaternion
    \param q2 The second quaternion
    \param alpha A value between 0 and 1
    \return The intermediate quaternion
    */
    MathTools::Quaternion  slerp(const MathTools::Quaternion& q1, const MathTools::Quaternion& q2, float alpha);

    float  fmod(float value, float boundLow, float boundHigh);

    float  angleMod2PI(float value);

    float  angleModPI(float value);

    float  angleModX(float value, float center);

    float  Lerp(float a, float b, float f);

    float  ILerp(float a, float b, float f);

    float  AngleLerp(float a, float b, float f);

    float  AngleDelta(float angle1, float angle2);

    /************************************************************************/
    /* SPHERICAL COORDINATES                                                */
    /************************************************************************/
    struct SphericalCoord
    {
        float r, phi, theta;
    };

    SphericalCoord  toSphericalCoords(const Eigen::Vector3f& pos);
    Eigen::Vector3f  toPosition(const SphericalCoord& sc);


    /************************************************************************/
    /* CONVERTIONS                                                          */
    /************************************************************************/

    /*!
        Convert rpy values to a 3x3 rotation matrix and store it to the rotational component of the given homogeneous matrix.
        The translation is set to zero.
    */
    void  rpy2eigen4f(float r, float p, float y, Eigen::Matrix4f& m);
    Eigen::Matrix4f  rpy2eigen4f(float r, float p, float y);
    Eigen::Matrix4f  rpy2eigen4f(const Eigen::Vector3f& rpy);

    Eigen::Matrix3f  rpy2eigen3f(float r, float p, float y);
    Eigen::Matrix3f  rpy2eigen3f(const Eigen::Vector3f& rpy);

    void  posrpy2eigen4f(const float x[6], Eigen::Matrix4f& m);
    void  posrpy2eigen4f(const Eigen::Vector3f& pos, const Eigen::Vector3f& rpy, Eigen::Matrix4f& m);
    void  posrpy2eigen4f(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Matrix4f& m);
    Eigen::Matrix4f  posrpy2eigen4f(const Eigen::Vector3f& pos, const Eigen::Vector3f& rpy);
    Eigen::Matrix4f  posrpy2eigen4f(float x, float y, float z, float roll, float pitch, float yaw);
    Eigen::Matrix4f  posquat2eigen4f(float x, float y, float z, float qx, float qy, float qz, float qw);
    Eigen::Matrix3f  quat2eigen3f(float qx, float qy, float qz, float qw);

    /*!
        Convert homogeneous matrix to translation and rpy rotation.
        \param m The matrix to be converted
        \param x The result is stored in this float array (x,y,z,roll,pitch,yaw)
    */
    void  eigen4f2rpy(const Eigen::Matrix4f& m, float x[6]);
    void  eigen4f2rpy(const Eigen::Matrix4f& m, Eigen::Vector3f& storeRPY);
    Eigen::Vector3f  eigen4f2rpy(const Eigen::Matrix4f& m);
    Eigen::Vector3f  eigen3f2rpy(const Eigen::Matrix3f& m);
    Eigen::Matrix<float, 6, 1>  eigen4f2posrpy(const Eigen::Matrix4f& m);

    /*!
        Convert quaternion values to a 3x3 rotation matrix and store it to the rotational component of the result.
        The translational part of m is zero
        \return Homogeneous matrix representing the rotation of q.
    */
    Eigen::Matrix4f  quat2eigen4f(float x, float y, float z, float w);
    Eigen::Matrix4f  quat2eigen4f(const Quaternion q);
    void  quat2eigen4f(float x, float y, float z, float w, Eigen::Matrix4f& m);
    void  quat2eigen4f(const Quaternion q, Eigen::Matrix4f& m);

    Quaternion  eigen4f2quat(const Eigen::Matrix4f& m);
    void  eigen4f2quat(const Eigen::Matrix4f& m, Quaternion& q);

    Eigen::Vector3f  getTranslation(const Eigen::Matrix4f& m);
    void  getTranslation(const Eigen::Matrix4f& m, Eigen::Vector3f& v);

    void  eigen4f2axisangle(const Eigen::Matrix4f& m, Eigen::Vector3f& storeAxis, float& storeAngle);
    Eigen::Matrix4f  axisangle2eigen4f(const Eigen::Vector3f& axis, float angle);
    void  axisangle2eigen4f(const Eigen::Vector3f& axis, float angle, Eigen::Matrix4f& storeResult);
    Eigen::Matrix3f  axisangle2eigen3f(const Eigen::Vector3f& axis, float angle);
    Quaternion  axisangle2quat(const Eigen::Vector3f& axis, float angle);

    /*!
        Compute the delta of two poses.
        \param m1 The first pose.
        \param m2 The second pose.
        \param storeDetalPos The position delta is stored here.
        \param storeDeltaRot The orientation delta is stored here [radian]
    */
    void  getDelta(const Eigen::Matrix4f& m1, const Eigen::Matrix4f& m2, float& storeDetalPos, float& storeDeltaRot);


    float  rad2deg(float rad);
    float  deg2rad(float deg);

    // [0,2PI]x[0,2PI]x[0,2PI]
    Eigen::Vector3f  quat2hopf(const Quaternion& q);
    Quaternion  hopf2quat(const Eigen::Vector3f& hopf);

    /************************************************************************/
    /* GEOMETRY                                                             */
    /************************************************************************/
    struct Plane
    {
        Plane()
        {
            p = Eigen::Vector3f::Zero();
            n = Eigen::Vector3f::UnitZ();
        }

        Plane(const Eigen::Vector3f& point, const Eigen::Vector3f& normal)
        {
            p = point;
            n = normal;
            n.normalize();
        }

        Plane(const Plane& plane)
        {
            this->p = plane.p;
            this->n = plane.n;
        }

        Eigen::Vector3f p;  // point
        Eigen::Vector3f n;  // normal (unit length)
    };

    //! Create a floor plane
    Plane  getFloorPlane();

    template<typename VectorT>
    struct BaseLine
    {
        BaseLine() {}

        BaseLine(const VectorT& point, const VectorT& dir)
        {
            p = point;
            d = dir;

            if (d.norm() > 1e-9)
            {
                d.normalize();
            }
        }

        BaseLine(const BaseLine<VectorT>& line)
            : p(line.p)
            , d(line.d)
        {
        }

        bool isValid() const
        {
            return d.norm() > 1e-9;
        }

        VectorT p;  // point
        VectorT d;  // direction (unit length)
    };

    typedef BaseLine<Eigen::Vector3f> Line;
    typedef BaseLine<Eigen::Vector2f> Line2D;

    struct Segment
    {
        Segment()
        {
            p0.setZero();
            p1.setZero();
        }
        Segment(const Eigen::Vector3f& point0, const Eigen::Vector3f& point1)
        {
            p0 = point0;
            p1 = point1;
        }
        Eigen::Vector3f p0;
        Eigen::Vector3f p1;
    };

    struct OOBB
    {
        OOBB();

        OOBB(const Eigen::Vector3f& minLocal, const Eigen::Vector3f& maxLocal, const Eigen::Matrix4f& globalPose);

        //! Returns the 8 bounding box points transformed to global frame
        std::vector<Eigen::Vector3f> getOOBBPoints() const;
        //! Returns the 12 segments of the bounding box (in global frame)
        std::vector<Segment> getSegments() const;

        void changeCoordSystem(const Eigen::Matrix4f& newGlobalPose);

        // the bounding box is defined via min and max values (in local frame)
        Eigen::Vector3f minBB;
        Eigen::Vector3f maxBB;

        Eigen::Matrix4f pose; // the transformation of the bounding box in global frame.
    };

    /*!
        Convenient structs for handling 3D/6D vertices, faces and convex hulls.
    */
    struct ContactPoint
    {
        Eigen::Vector3f p = Eigen::Vector3f::Zero();  // point
        Eigen::Vector3f n = Eigen::Vector3f::Zero();  // normal
        float force = 0;
    };

    struct TriangleFace
    {
        TriangleFace() = default;

        /**
         * Flips the orientation of the contained vertex and the normal.
         */
        void flipOrientation()
        {
            std::swap(id3, id1);
            normal *= -1.0f;
        }
        void set(unsigned int id1, unsigned int id2, unsigned int id3)
        {
            this->id1 = id1;
            this->id2 = id2;
            this->id3 = id3;
        }
        void setColor(unsigned int idColor1, unsigned int idColor2, unsigned int idColor3)
        {
            this->idColor1 = idColor1;
            this->idColor2 = idColor2;
            this->idColor3 = idColor3;
        }
        void setNormal(unsigned int idNormal1, unsigned int idNormal2, unsigned int idNormal3)
        {
            this->idNormal1 = idNormal1;
            this->idNormal2 = idNormal2;
            this->idNormal3 = idNormal3;
        }
        void setMaterial(unsigned int idMaterial)
        {
            this->idMaterial = idMaterial;
        }

        // id == position in vertex array
        unsigned int id1{UINT_MAX};
        unsigned int id2{UINT_MAX};
        unsigned int id3{UINT_MAX};

        // idColor == position in color array
        unsigned int idColor1{UINT_MAX};
        unsigned int idColor2{UINT_MAX};
        unsigned int idColor3{UINT_MAX};

        //idNormal == position in normal array
        unsigned int idNormal1{UINT_MAX};
        unsigned int idNormal2{UINT_MAX};
        unsigned int idNormal3{UINT_MAX};

        // idMaterial == position in material array
        unsigned int idMaterial{UINT_MAX};

        // per face normal (used when no idNormals are given)
        Eigen::Vector3f normal{0, 0, 0};
    };
    struct TriangleFace6D
    {
        int id[6];// position in vertice vector (x,y,z,nx,ny,nz)
        ContactPoint normal;

        // these values are set by the ConvexHull algorithm (see GraspStudio)
        float distNormZero;     // distance of facet to origin
        float distNormCenter;   // distance of facet to center of convex hull
        float distPlaneZero;    // distance of plane defined by facet to origin
        float distPlaneCenter;  // distance of plane defined by facet to center of convex hull
        float offset;           // offset value of facet, determined by qhull
    };

    enum IntersectionResult
    {
        eParallel,
        eNoIntersection,
        eIntersection
    };


    //! Get the projected point in 3D
    Eigen::Vector3f  projectPointToPlane(const Eigen::Vector3f& point, const Plane& plane);

    //! Get the intersection line of two planes. If planes are parallel, the resulting line is invalid, i.e. has a zero direction vector!
    Line  intersectPlanes(const Plane& p1, const Plane& p2);

    /*!
        Get the intersection of segment and plane.
        \param segment The segment
        \param plane The plane
        \param storeResult In case the intersection exists, the result is stored here
        \result If there is no intersection the result is eNoIntersection. In case the result is eIntersection, the resulting intersection point is stored in storeResult.
    */
    IntersectionResult  intersectSegmentPlane(const Segment& segment, const Plane& plane, Eigen::Vector3f& storeResult);

    /*!
    Intersect an object oriented bounding box (oobb) with a plane.
    \param oobb The oobb
    \param plane The plane
    \param storeResult In case an intersection exists, the intersection area is defined by these points
    \result If the oobb does not intersect eNoIntersection is returned, otherwise eIntersection.
    */
    IntersectionResult  intersectOOBBPlane(const OOBB& oobb, const Plane& plane, std::vector<Eigen::Vector3f>& storeResult);

    //! Returns nearest point to p on line l
    template<typename VectorT>
    inline VectorT nearestPointOnLine(const BaseLine<VectorT>& l, const VectorT& p)
    {
        if (!l.isValid())
        {
            return VectorT::Zero();
        }

        VectorT lp = p - l.p;

        float lambda = l.d.dot(lp);

        VectorT res = l.p + lambda * l.d;
        return res;
    }

    //! Returns nearest point to p on segment given by the two end points
    template<typename VectorT>
    inline VectorT nearestPointOnSegment(const VectorT& firstEndPoint, const VectorT& secondEndPoint, const VectorT& p)
    {
        VectorT direction = secondEndPoint - firstEndPoint;
        direction /= direction.norm();
        const BaseLine<VectorT> l(firstEndPoint, direction);
        VR_ASSERT(l.isValid());

        VectorT onLine = nearestPointOnLine<VectorT>(l, p);
        double alpha = (onLine - firstEndPoint).dot(direction);

        // point not on segment, below first end point
        if (alpha < 0)
        {
            return firstEndPoint;
        }

        // point not on segment, above second end point
        if (alpha > (secondEndPoint - firstEndPoint).norm())
        {
            return secondEndPoint;
        }

        return onLine;
    }

    //! Returns the distance of vector p to line l
    template<typename VectorT>
    inline float distPointLine(const BaseLine<VectorT>& l, const VectorT& p)
    {
        if (!l.isValid())
        {
            return -1.0f;
        }

        VectorT p2 = nearestPointOnLine<VectorT>(l, p);
        return (p2 - p).norm();
    }

    //! Returns the distance of vector p to segment given by the two end points
    template<typename VectorT>
    inline float distPointSegment(const VectorT& firstEndPoint, const VectorT& secondEndPoint, const VectorT& p)
    {
        return (p - nearestPointOnSegment(firstEndPoint, secondEndPoint, p)).norm();
    }

    //! Check if three points are collinear
    bool  collinear(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3);

    /*!
        assuming all points to be in a plane. Returns normal of this plane.
    */
    Eigen::Vector3f  findNormal(const std::vector<Eigen::Vector3f>& points);

    //! Get the projected point in 2D (local coordinate system of the plane)
    Eigen::Vector2f  projectPointToPlane2D(const Eigen::Vector3f& point, const Plane& plane);

    //! Get the corresponding point in 3D
    Eigen::Vector3f  planePoint3D(const Eigen::Vector2f& pointLocal, const Plane& plane);

    float  getDistancePointPlane(const Eigen::Vector3f& point, const Plane& plane);

    /*!
        This method can be used to multiply a 3d position with a matrix
        result = m * pos
    */
    Eigen::Vector3f  transformPosition(const Eigen::Vector3f& pos, const Eigen::Matrix4f& m);

    /*!
        This method can be used to multiply a 2d position with a matrix
        result = m * pos
    */
    Eigen::Vector2f  transformPosition(const Eigen::Vector2f& pos, const Eigen::Matrix4f& m);

    //! Get a random point inside the triangle that is spanned by v1, v2 and v3
    Eigen::Vector3f  randomPointInTriangle(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3);

    /*!
        Returns true, if point is on the side of plane in which the normal vector is pointing.
    */
    bool  onNormalPointingSide(const Eigen::Vector3f& point, const Plane& p);

    /*!
        Returns angle between v1 and v2 [rad].
    */
    float  getAngle(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

    /*!
        This method unites the translational and rotational difference of two Cartesian poses to one metric value.
        The translational distance is the norm in mm (counts 1)
        The rotational distance is the approximated angle between the two orientations in degrees (counts 3, or whatever you specify with rotInfluence)
        In the standard setting -> 3mm equals 1 degree in this metric
        The angle is approximated by replacing the costly eq alpha=2*acos(q0) with a linear term: alpha = 180-(q0+1)*90
    */
    float  getCartesianPoseDiff(const Eigen::Matrix4f& p1, const Eigen::Matrix4f& p2, float rotInfluence = 3.0f);

    /*!
        * \brief getPoseDiff Computes the translational and rotational difference between two poses.
        * \param p1 First Pose.
        * \param p2 Second Pose.
        * \param storePosDiff The translational absolute distance between p1 and p2 is stored here
        * \param storeRotDiffRad The rotational absolute distance between p1 and p2 is stored here (radian)
        * \return
        */
    void  getPoseDiff(const Eigen::Matrix4f& p1, const Eigen::Matrix4f& p2, float& storePosDiff, float& storeRotDiffRad);

    float  getTriangleArea(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c);

    /************************************************************************/
    /* CONVEX HULLS                                                         */
    /* More convex hull methods can be found in the GarspStudio lib         */
    /************************************************************************/

    struct Segment2D
    {
        int id1;
        int id2;
    };

    struct ConvexHull2D
    {
        std::vector<Eigen::Vector2f> vertices;
        std::vector<Segment2D> segments;
    };
    typedef std::shared_ptr<ConvexHull2D> ConvexHull2DPtr;

    struct ConvexHull3D
    {
        std::vector<Eigen::Vector3f> vertices;
        std::vector<TriangleFace> faces;
        float volume;

        Eigen::Vector3f center;
        float maxDistFacetCenter; // maximum distance of faces to center
    };
    typedef std::shared_ptr<ConvexHull3D> ConvexHull3DPtr;

    struct ConvexHull6D
    {
        std::vector<ContactPoint>   vertices;
        std::vector<TriangleFace6D> faces;
        float volume;
        ContactPoint center;
    };
    typedef std::shared_ptr<ConvexHull6D> ConvexHull6DPtr;


    // Copyright 2001, softSurfer (www.softsurfer.com)
    // This code may be freely used and modified for any purpose
    // providing that this copyright notice is included with it.
    // SoftSurfer makes no warranty for this code, and cannot be held
    // liable for any real or imagined damage resulting from its use.
    // Users of this code must verify correctness for their application.
    // isLeft(): tests if a point is Left|On|Right of an infinite line.
    //    Input:  three points P0, P1, and P2
    //    Return: >0 for P2 left of the line through P0 and P1
    //            =0 for P2 on the line
    //            <0 for P2 right of the line
    //    See: the January 2001 Algorithm on Area of Triangles
    inline float isLeft(Eigen::Vector2f P0, Eigen::Vector2f P1, Eigen::Vector2f P2)
    {
        return (P1(0) - P0(0)) * (P2(1) - P0(1)) - (P2(0) - P0(0)) * (P1(1) - P0(1));
    }

    ConvexHull2DPtr  createConvexHull2D(const std::vector< Eigen::Vector2f >& points);
    Eigen::Vector2f  getConvexHullCenter(ConvexHull2DPtr ch);
    bool  isInside(const Eigen::Vector2f& p, ConvexHull2DPtr hull);

    /*!
        Sort points according to their first coordinate. If two points share the first coord, the second one is used to decide which is "smaller".
    */
    std::vector< Eigen::Vector2f > sortPoints(const std::vector< Eigen::Vector2f >& points);



    /************************************************************************/
    /* BASIS HELPERS                                                        */
    /************************************************************************/
    bool  ensureOrthonormalBasis(Eigen::Vector3f& x, Eigen::Vector3f& y, Eigen::Vector3f& z);


    //! perform the Gram-Schmidt method for building an orthonormal basis
    bool  GramSchmidt(std::vector< Eigen::VectorXf >& basis);

    //! Searches a vector that is linearly independent of the given basis.
    bool  randomLinearlyIndependentVector(const std::vector< Eigen::VectorXf > basis, Eigen::VectorXf& storeResult);

    //! Checks if m contains a col vector equal to v (distance up to 1e-8 are considered as equal)
    bool  containsVector(const Eigen::MatrixXf& m, const Eigen::VectorXf& v);


    /*!
        Computes the matrix describing the basis transformation from basis formed by vectors in basisSrc to basisDst.
        So v_{Src} can be transformed to v_{Dst}, given as linear combination of basisDst vectors, by
        v_{Dst} = T v_{Src} with T is the BasisTransformationMatrix given by this method.
        \param basisSrc The initial basis vectors.
        \param basisDst The final basis vectors.
        \return The transformation matrix T.
    */
    Eigen::MatrixXf  getBasisTransformation(const std::vector< Eigen::VectorXf >& basisSrc, const std::vector< Eigen::VectorXf >& basisDst);

    /*!
        Computes the matrix describing the basis transformation from basis formed by vectors in basisSrc to basisDst.
        So v_{Src} can be transformed to v_{Dst}, given as linear combination of basisDst vectors, by
        v_{Dst} = T v_{Src} with T is the BasisTransformationMatrix given by this method.
        \param basisSrc The column vectors are the initial basis vectors
        \param basisDst The column vectors are the final basis vectors.
        \return The transformation matrix T.
    */
    Eigen::MatrixXf  getBasisTransformation(const Eigen::MatrixXf& basisSrc, const Eigen::MatrixXf& basisDst);

    Eigen::VectorXf  getPermutation(const Eigen::VectorXf& inputA, const Eigen::VectorXf& inputB, unsigned int i);
    /************************************************************************/
    /* HELPERS and IO                                                       */
    /************************************************************************/

    int  pow_int(int a, int b);

    /*!
        Returns the Pseudo inverse matrix.
    */
    Eigen::MatrixXf  getPseudoInverse(const Eigen::MatrixXf& m, float tol = 1e-5f);
    Eigen::MatrixXd  getPseudoInverseD(const Eigen::MatrixXd& m, double tol = 1e-5);
    /*!
        Returns the damped Pseudo inverse matrix.
    */
    Eigen::MatrixXf  getPseudoInverseDamped(const Eigen::MatrixXf& m, float lambda = 1.0f);
    Eigen::MatrixXd  getPseudoInverseDampedD(const Eigen::MatrixXd& m, double lambda = 1.0);

    /*!
        Check if all entries of v are valid numbers (i.e. all entries of v are not NaN and not INF)
    */
    bool  isValid(const Eigen::MatrixXf& v);
    void  print(const ContactPoint& p);
    void  print(const std::vector<ContactPoint>& points);
    void  print(const Eigen::VectorXf& v, bool endline = true);
    void  printMat(const Eigen::MatrixXf& m, bool endline = true);
    void  print(const std::vector<float>& v, bool endline = true);
    std::string  getTransformXMLString(const Eigen::Matrix4f& m, int tabs, bool skipMatrixTag = false);
    std::string  getTransformXMLString(const Eigen::Matrix4f& m, const std::string& tabs, bool skipMatrixTag = false);
    std::string  getTransformXMLString(const Eigen::Matrix3f& m, int tabs, bool skipMatrixTag = false);
    std::string  getTransformXMLString(const Eigen::Matrix3f& m, const std::string& tabs, bool skipMatrixTag = false);
    void  convertMM2M(const std::vector<ContactPoint> points, std::vector<ContactPoint>& storeResult);
}