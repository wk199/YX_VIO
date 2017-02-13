/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include "Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include <gtsam/geometry/Pose3.h>

using namespace gtsam;

namespace ORB_SLAM
{

class LoopClosing;

class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP, int nIterations = 5, bool *pbStopFlag=NULL);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL);
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag=NULL);
    int static PoseOptimization(Frame* pFrame);

    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, g2o::Sim3 &Scurw,
                                       LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       std::map<KeyFrame*, set<KeyFrame*> > &LoopConnections);


    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, float th2 = 10);
    //static void TwoFrameBundleAdjustmentUseISAM(KeyFrame* pKF1, KeyFrame* pKF2);
    //static void LocalBundleAdjustmentUseISAM2(KeyFrame* pKF, /*bool *pbStopFlag = NULL, */Map* pMap);
	static void LocalBundleAdjustmentUseIMU(KeyFrame* pKF, IMUMeasurementList* imu, bool *pbStopFlag=NULL);
    static void LocalBundleAdjustmentUseIMUTemporal(KeyFrame* pKF, IMUMeasurementList* imu, Map* pMap, bool *OptStopFlag=NULL, bool *pbStopFlag=NULL );
    static void InitialBAUseIMU(KeyFrame* preKF, KeyFrame* curKF, IMUMeasurementList* imu);

    static void PoseOptimizationUseImu(KeyFrame* LastKF, Frame *pLastFrame, Frame *pCurrentFrame,  Map* pMap,IMUMeasurementList* imu, bool ifKF);
    static bool InitialScalityAndVelocity(Map* mpMap, IMUMeasurementList* imu, Frame* pLastFrame, cv::Mat &e_gw);
    ////////////////////////////////////////////////////////
    //
    static gtsam::Pose3 toGTPose3(const cv::Mat& _m);
    static cv::Mat Pose3TocvMat(const gtsam::Pose3 _p);
    static cv::Mat Pose3TocvMat(const boost::optional<gtsam::Pose3> _p);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
