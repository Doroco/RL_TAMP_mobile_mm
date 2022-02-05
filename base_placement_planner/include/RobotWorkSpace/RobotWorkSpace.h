#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <deque>
#include <queue>
#include <map>

#include <Eigen/Core>

// 추상함수 인터페이스 관리
namespace RobotWorkSpace
{
    class PoseQualityMeasurement;
    class PoseQualityManipulability;
    class PoseQualityExtendedManipulability;
    class CompressionBZip2;
    class WorkspaceData;
    class WorkspaceDataArray;
    class WorkspaceRepresentation;
    class Reachability;
    class Grasp;
    class GraspSet;
    class WorkspaceGrid;

    typedef std::shared_ptr<PoseQualityMeasurement> PoseQualityMeasurementPtr;
    typedef std::shared_ptr<PoseQualityManipulability> PoseQualityManipulabilityPtr;
    typedef std::shared_ptr<PoseQualityExtendedManipulability> PoseQualityExtendedManipulabilityPtr;
    typedef std::shared_ptr<CompressionBZip2> CompressionBZip2Ptr;
    typedef std::shared_ptr<WorkspaceData> WorkspaceDataPtr;
    typedef std::shared_ptr<WorkspaceDataArray> WorkspaceDataArrayPtr;
    typedef std::shared_ptr<WorkspaceRepresentation> WorkspaceRepresentationPtr;
    typedef std::shared_ptr<Reachability> ReachabilityPtr;
    typedef std::shared_ptr<Grasp> GraspPtr;
    typedef std::shared_ptr<GraspSet> GraspSetPtr;
    typedef std::shared_ptr<WorkspaceGrid> WorkspaceGridPtr;
    
    // Shpere discrete
    typedef std::multimap< const std::vector< double >*, const std::vector< double >* > MultiMapPtr;
    typedef std::map< const std::vector< double >*, double > MapVecDoublePtr;
    typedef std::multimap< std::vector< double >, std::vector< double > > MultiMap;
    typedef std::map< std::vector< double >, double > MapVecDouble;
    typedef std::vector<std::vector<double> > VectorOfVectors;
    typedef std::vector<std::pair< std::vector< double >, const std::vector< double >* > > MultiVector;

} // namespace RobotWorkSpace
