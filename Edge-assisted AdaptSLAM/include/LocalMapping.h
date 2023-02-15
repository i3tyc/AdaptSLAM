/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Settings.h"

#include <mutex>
////////////////////////////CommSLAM//////////////////////
#include "TcpSocket.h"
#include <thread>
#include "concurrentqueue.h"
#include "blockingconcurrentqueue.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include "ORBVocabulary.h"

namespace ORB_SLAM3
{

class System;
class Tracking;
class LoopClosing;
class Atlas;
////////////////////CommSLAM//////////////////////////////////////
class Uncertainty;

class LocalMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ////////////////////CommSLAM//////////////////////////////////////
    LocalMapping(System* pSys, Atlas* pAtlas, KeyFrameDatabase* pKFDB, ORBVocabulary* pVoc, Uncertainty* pUncertainty, const float bMonocular, bool bInertial, string RunType, const string &_strSeqName=std::string());
    
    // Edge-SLAM: TCP
    void static tcp_receive(moodycamel::ConcurrentQueue<std::string>* messageQueue, TcpSocket* socketObject, unsigned int maxQueueSize, std::string name);
    void static tcp_send(moodycamel::BlockingConcurrentQueue<std::string>* messageQueue, TcpSocket* socketObject, std::string name);
    
    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

    // Thread Synch
    void RequestStop();
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();
    KeyFrame* GetCurrKF();
    
    
    ////////////////////////CommSLAM/////////////////////
    void PostLoadKFandMP(KeyFrame* pKF);
    void PreSaveKFandMP(KeyFrame* pKF);
    void PostLoadKFandMPSet(map<long unsigned int, KeyFrame*> StoreKFid);
    map<long unsigned int, MapPoint*> GetmpMPid();
    map<long unsigned int, KeyFrame*> GetmpKFid();
    
    std::mutex mMutexImuInit;

    Eigen::MatrixXd mcovInertial;
    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mScale;
    double mInitTime;
    double mCostTime;

    unsigned int mInitSect;
    unsigned int mIdxInit;
    unsigned int mnKFs;
    double mFirstTs;
    int mnMatchesInliers;

    // For debugging (erase in normal mode)
    int mInitFr;
    int mIdxIteration;
    string strSequence;

    bool mbNotBA1;
    bool mbNotBA2;
    bool mbBadImu;

    bool mbWriteStats;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;

    //////////////CommSLAM///////////
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;
    void AddKFSet(KeyFrame* kf);
    void AddMPSet(MapPoint* mp);
    void AddCameraSet(GeometricCamera* cam);
    void AssignMap(KeyFrame* KF, int pMapId);
    /////////////////////CommSLAM//////////////////////
    moodycamel::BlockingConcurrentQueue<std::string> client_uplink_queue;
    moodycamel::ConcurrentQueue<std::string> client_downlink_queue;
    moodycamel::BlockingConcurrentQueue<std::string> server_downlink_queue;
    moodycamel::ConcurrentQueue<std::string> server_uplink_queue;
    map<long unsigned int, KeyFrame*> mpKFid;
    map<long unsigned int, MapPoint*> mpMPid;
    map<unsigned int, GeometricCamera*> mpCamId;
    Atlas* mpAtlas;
    int nflag;
    KeyFrame *mpCurrentKeyFramePre;
    
    
    
    TcpSocket* uplink_socket;
    TcpSocket* downlink_socket;
    
    std::thread* uplink_thread ;
    std::thread* downlink_thread; 
    
#ifdef REGISTER_TIMES
    vector<double> vdKFInsert_ms;
    vector<double> vdMPCulling_ms;
    vector<double> vdMPCreation_ms;
    vector<double> vdLBA_ms;
    vector<double> vdKFCulling_ms;
    vector<double> vdLMTotal_ms;


    vector<double> vdLBASync_ms;
    vector<double> vdKFCullingSync_ms;
    vector<int> vnLBA_edges;
    vector<int> vnLBA_KFopt;
    vector<int> vnLBA_KFfixed;
    vector<int> vnLBA_MPs;
    int nLBA_exec;
    int nLBA_abort;
#endif
protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();
    void KeyFrameCulling();

    System *mpSystem;

    bool mbMonocular;
    bool mbInertial;

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

///////////////////////CommSLAM//////////////////////////
    Uncertainty* mpUncertainty;
    

    string mRunType;

    
    
    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    void InitializeIMU(float priorG = 1e2, float priorA = 1e6, bool bFirst = false);
    void ScaleRefinement();

    bool bInitializing;

    Eigen::MatrixXd infoInertial;
    int mNumLM;
    int mNumKFCulling;

    float mTinit;

    int countRefinement;

    //DEBUG
    ofstream f_lm;

    };

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
