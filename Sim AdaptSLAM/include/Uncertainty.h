#include "KeyFrame.h"
#include "MapPoint.h"
#include "Map.h"
#include <Eigen/Core>
#ifndef UNCERNTAINTY_H
#define UNCERNTAINTY_H
namespace ORB_SLAM3
{
class Uncertainty
{
public:
    Eigen::MatrixXf mLocalLaplacianMatrix;
    Eigen::MatrixXf mGlobalLaplacianMatrix;
    Eigen::MatrixXf mLocalInertialLaplacianMatrix;
    Eigen::MatrixXf mGlobalInertialLaplacianMatrix;
    Eigen::MatrixXf mMergeVisualLaplacianMatrix;
    vector<float> mvLocalDOptimal;
    vector<float> mvGlobalDOptimal;
    vector<float> mvLocalInertialDOptimal;
    vector<float> mvGlobalInertialDOptimal;
    vector<float> mvMergeVisualDOptimal;
    void LocalMapUncertainty(list<KeyFrame*> &lLocalKeyFrames,Map* pMap);
    void LocalInertialUncertainty(vector<KeyFrame*> &vpOptimizableKFs);
    void GlobalMapUncertainty(Map* pMap);
    void GlobalInertialUncertainty(Map* pMap);
    void MergeVisualUncertainty(KeyFrame* pMainKF,vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF); 
    float UpdateUncertainty(vector<KeyFrame*> vpKFs,KeyFrame* pKF);
    float UpdateUncertaintyList(list<KeyFrame*> vpKFs,KeyFrame* pKF);
};
}

#endif 
