#include "Uncertainty.h"
#include<Eigen/StdVector>
#include<Eigen/Core>
#include <Eigen/LU> 
#include<Eigen/Dense>
//#include<Eigen/CholmodSupport>

namespace ORB_SLAM3
{
    
void Uncertainty::LocalMapUncertainty(list<KeyFrame*> &lLocalKeyFrames,Map* pMap)
{
    /*
    list<KeyFrame *> lLocalKeyFrames;
    lLocalKeyFrames.push_back(pKF);

    const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
        KeyFrame *pKFi = vNeighKFs[i];
        if (!pKFi->isBad() && pKFi->GetMap() == pMap)
            lLocalKeyFrames.push_back(pKFi);
    }
    */
    
    
    int SIZE_ =lLocalKeyFrames.size();
    mLocalLaplacianMatrix.resize(SIZE_,SIZE_);
    int i=0;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(),lend=lLocalKeyFrames.end();lit!=lend;lit++)
    {
        int j=i-1;
        for(list<KeyFrame*>::iterator lit2=lit, lend2=lLocalKeyFrames.end();lit2!=lend2;lit2++)
        {
            j++;
            if (lit!=lit2)
            {
                float temp=-((*lit)->GetWeight((*lit2)))*0.0002;
                mLocalLaplacianMatrix(i, j)= temp;
                mLocalLaplacianMatrix(j, i)= temp;
            }
            else
            {
                 mLocalLaplacianMatrix(i, j)=0;
            }
        }
        i++;
    }
    
    i=0;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(),lend=lLocalKeyFrames.end();lit!=lend;lit++)
    {
        for(int j=0;j<lLocalKeyFrames.size();j++)
        {
            if (i!=j)
            {
                mLocalLaplacianMatrix(i, i)-=mLocalLaplacianMatrix(i, j);
            }
        }
        i++;
    }
    
 
    Eigen::MatrixXf ReducedLaplacianMatrix;
    ReducedLaplacianMatrix.resize(SIZE_-1,SIZE_-1);
    ReducedLaplacianMatrix=mLocalLaplacianMatrix.block(0,0,SIZE_-1,SIZE_-1);
    
    
    //cerr<<"Size"<<ReducedLaplacianMatrix<<endl;   
    float DOptimal= log(ReducedLaplacianMatrix.determinant());
    //cerr<<"Local Map"<<DOptimal<<endl;
    mvLocalDOptimal.push_back(DOptimal);
    
}

void Uncertainty::GlobalMapUncertainty(Map* pMap)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    int SIZE_=vpKFs.size();
    mGlobalLaplacianMatrix.resize(SIZE_,SIZE_);

    for(size_t i=0;i<SIZE_;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        
        for(size_t j=i;j<SIZE_;j++)
        {
            if(i!=j)
            {
                KeyFrame* pKF2 = vpKFs[j];
                //float temp=-pKF->GetWeight(pKF2)*0.0005;
                float temp=-pKF->GetWeight(pKF2)*0.0002;
                mGlobalLaplacianMatrix(i, j)= temp;
                mGlobalLaplacianMatrix(j, i)= temp;
            }
            else
            {
                mGlobalLaplacianMatrix(i,j)=0;
            }
        }

    }
    for(size_t i=0;i<SIZE_;i++)
    {
        for(size_t j=0;j<SIZE_;j++)
        {
            if(i!=j)
            {
                mGlobalLaplacianMatrix(i, i)-=mGlobalLaplacianMatrix(i, j);
            }
        }
    }
    
    vector<int> NonZeros;
    for(size_t i=0;i<SIZE_;i++)
    {
        if (mGlobalLaplacianMatrix(i, i)>0.002)
        {
            NonZeros.push_back(i);
        }
    }
    int NonZerosSize=NonZeros.size();
    Eigen::MatrixXf NonZerosCol(SIZE_,NonZerosSize);
    for(size_t i=0;i<NonZeros.size();i++)
    {
        NonZerosCol.col(i)=mGlobalLaplacianMatrix.col(NonZeros[i]);
    }
    Eigen::MatrixXf NonZerosMatrix(NonZerosSize,NonZerosSize);
    for(size_t i=0;i<NonZeros.size();i++)
    {
        NonZerosMatrix.row(i)=NonZerosCol.row(NonZeros[i]);
    }
    Eigen::MatrixXf ReducedLaplacianMatrix;
    ReducedLaplacianMatrix.resize(NonZerosSize-1,NonZerosSize-1);
    ReducedLaplacianMatrix=NonZerosMatrix.block(0,0,NonZerosSize-1,NonZerosSize-1);
    

    cerr<<"Size"<<ReducedLaplacianMatrix.size()<<endl;   
    float DOptimal= log(ReducedLaplacianMatrix.determinant());
    cerr<<"Global Map"<<DOptimal<<endl;
    mvGlobalDOptimal.push_back(DOptimal);

}

void Uncertainty::LocalInertialUncertainty(vector<KeyFrame*> &vpOptimizableKFs)
{
    double InertialEdgeWeight=0.01;
    int N=vpOptimizableKFs.size();
    mLocalInertialLaplacianMatrix.resize(N,N);

    for(int i=0; i<N; i++)
    {
        KeyFrame* pKF = vpOptimizableKFs[i];
        for(size_t j=i;j<N;j++)
        {
            if(i!=j)
            {
                KeyFrame* pKF2 = vpOptimizableKFs[j];
                float temp=-pKF->GetWeight(pKF2)*0.0002;
                mLocalInertialLaplacianMatrix(i, j)= temp;
                mLocalInertialLaplacianMatrix(j, i)= temp;
            }
            else
            {
                mLocalInertialLaplacianMatrix(i,j)=0;
            }
        }
    }
    for(int i=0; i<N-1; i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];
        
        if(pKFi->mPrevKF)
        {
            if(pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated)
            {
                mLocalInertialLaplacianMatrix(i, i+1)-= InertialEdgeWeight;
                mLocalInertialLaplacianMatrix(i+1, i)-= InertialEdgeWeight;
            }
        }
    }
    
    for(size_t i=0;i<N;i++)
    {
        for(size_t j=0;j<N;j++)
        {
            if(i!=j)
            {
                mLocalInertialLaplacianMatrix(i, i)-=mLocalInertialLaplacianMatrix(i, j);
            }
        }
    }
    
    
    Eigen::MatrixXf ReducedLaplacianMatrix;
    ReducedLaplacianMatrix.resize(N-1,N-1);
    ReducedLaplacianMatrix=mLocalInertialLaplacianMatrix.block(0,0,N-1,N-1);
    

    cerr<<"Size"<<ReducedLaplacianMatrix.size()<<endl;   
    float DOptimal= log(ReducedLaplacianMatrix.determinant());
    cerr<<"Local Inertial"<<DOptimal<<endl;
    mvLocalInertialDOptimal.push_back(DOptimal);
}


void Uncertainty::GlobalInertialUncertainty(Map* pMap)
{
    double InertialEdgeWeight=0.01;


    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    int SIZE_=vpKFs.size();
    mGlobalInertialLaplacianMatrix.resize(SIZE_,SIZE_);
    
    map<int, int> m;
    for(size_t i=0; i<SIZE_; i++)
    {
        KeyFrame* pKF = vpKFs[i];
        m.insert(pair<int, int>(pKF->mnId,i));
        
        for(size_t j=i;j<SIZE_;j++)
        {
            if(i!=j)
            {
                KeyFrame* pKF2 = vpKFs[j];
                float temp=-pKF->GetWeight(pKF2)*0.0002;
                mGlobalInertialLaplacianMatrix(i, j)= temp;
                mGlobalInertialLaplacianMatrix(j, i)= temp;
            }
            else
            {
                mGlobalInertialLaplacianMatrix(i,j)=0;
            }
        }
    }
    

    for(size_t i=0; i<SIZE_; i++)
    {
        KeyFrame* pKF = vpKFs[i];
        
        int nIndex=m[pKF->mnId];

        if(pKF->mPrevKF)
        {
            if(pKF->bImu && pKF->mPrevKF->bImu && pKF->mpImuPreintegrated)
            {
                if(m.count(pKF->mPrevKF->mnId))
                {
                    mGlobalInertialLaplacianMatrix(nIndex, m[pKF->mPrevKF->mnId])-= InertialEdgeWeight;
                    mGlobalInertialLaplacianMatrix(m[pKF->mPrevKF->mnId], nIndex)-= InertialEdgeWeight;
                }
            }
        }
    } 

    for(size_t i=0;i<SIZE_;i++)
    {
        for(size_t j=0;j<SIZE_;j++)
        {
            if(i!=j)
            {
                mGlobalInertialLaplacianMatrix(i, i)-=mGlobalInertialLaplacianMatrix(i, j);
            }
        }
    }
    
    vector<int> NonZeros;
    for(size_t i=0;i<SIZE_;i++)
    {
        if (mGlobalInertialLaplacianMatrix(i, i)>0.002)
        {
            NonZeros.push_back(i);
        }
    }
    int NonZerosSize=NonZeros.size();
    Eigen::MatrixXf NonZerosCol(SIZE_,NonZerosSize);
    for(size_t i=0;i<NonZeros.size();i++)
    {
        NonZerosCol.col(i)=mGlobalInertialLaplacianMatrix.col(NonZeros[i]);
    }
    Eigen::MatrixXf NonZerosMatrix(NonZerosSize,NonZerosSize);
    for(size_t i=0;i<NonZeros.size();i++)
    {
        NonZerosMatrix.row(i)=NonZerosCol.row(NonZeros[i]);
    }
    Eigen::MatrixXf ReducedLaplacianMatrix;
    ReducedLaplacianMatrix.resize(NonZerosSize-1,NonZerosSize-1);
    ReducedLaplacianMatrix=NonZerosMatrix.block(0,0,NonZerosSize-1,NonZerosSize-1);
    
    cerr<<"Size"<<ReducedLaplacianMatrix.size()<<endl;   
    float DOptimal= log(ReducedLaplacianMatrix.determinant());
    cerr<<"Global Inertial"<<DOptimal<<endl;
    mvGlobalInertialDOptimal.push_back(DOptimal);
    
    
}

void Uncertainty::MergeVisualUncertainty(KeyFrame* pMainKF,vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF)
{
    Map *pCurrentMap = pMainKF->GetMap();
    vector<KeyFrame *> vpAllKeyFrames;
    int nNumAjustKF=0;
    int nNumFixedKF=0;
    for (KeyFrame *pKFi : vpAdjustKF)
    {
        if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
        {
            continue;
        }
        nNumFixedKF++;
        vpAllKeyFrames.push_back(pKFi);
    }
    for (KeyFrame *pKFi : vpFixedKF)
    {
        if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
        {
            continue;
        }
        nNumAjustKF++;
        vpAllKeyFrames.push_back(pKFi);
    }
    int SIZE_;
    SIZE_=nNumAjustKF+nNumFixedKF;
    
    mMergeVisualLaplacianMatrix.resize(SIZE_,SIZE_);
    
    for(size_t i=0;i<SIZE_;i++)
    {
        KeyFrame* pKF = vpAllKeyFrames[i];
        
        for(size_t j=i;j<SIZE_;j++)
        {
            if(i!=j)
            {
                KeyFrame* pKF2 = vpAllKeyFrames[j];
                float temp=-pKF->GetWeight(pKF2)*0.0002;
                mMergeVisualLaplacianMatrix(i, j)= temp;
                mMergeVisualLaplacianMatrix(j, i)= temp;
            }
            else
            {
                mMergeVisualLaplacianMatrix(i,j)=0;
            }
        }

    }
    
    for(size_t i=0;i<SIZE_;i++)
    {
        for(size_t j=0;j<SIZE_;j++)
        {
            if(i!=j)
            {
                mMergeVisualLaplacianMatrix(i, i)-=mMergeVisualLaplacianMatrix(i, j);
            }
        }
    }
    
    Eigen::MatrixXf ReducedLaplacianMatrix;
    ReducedLaplacianMatrix.resize(nNumAjustKF-1,nNumAjustKF-1);
    ReducedLaplacianMatrix=mMergeVisualLaplacianMatrix.block(0,0,nNumAjustKF-1,nNumAjustKF-1);
    
    cerr<<"Size"<<ReducedLaplacianMatrix.size()<<endl;   
    float DOptimal= log(ReducedLaplacianMatrix.determinant());
    cerr<<"Merge Visual"<<DOptimal<<endl;
    mvMergeVisualDOptimal.push_back(DOptimal);
}


float Uncertainty::UpdateUncertainty(vector<KeyFrame*> vpKFs,KeyFrame* pKF)
{
    int SIZE_=vpKFs.size();
    Eigen::MatrixXf mLaplacianMatrix;
    mLaplacianMatrix.resize(SIZE_+1,SIZE_+1);

    for(size_t i=0;i<SIZE_;i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        
        for(size_t j=i;j<SIZE_;j++)
        {
            if(i!=j)
            {
                KeyFrame* pKF2 = vpKFs[j];
                //float temp=-pKF->GetWeight(pKF2)*0.0005;
                float temp=-pKFi->GetWeight(pKF2)*0.0002;
                mLaplacianMatrix(i, j)= temp;
                mLaplacianMatrix(j, i)= temp;
            }
            else
            {
                mLaplacianMatrix(i,j)=0;
            }
        }
        
        float temp=-pKFi->GetWeight(pKF)*0.0002;
        mLaplacianMatrix(i, SIZE_)= temp;
        mLaplacianMatrix(SIZE_, i)= temp;

    }
    mLaplacianMatrix(SIZE_, SIZE_)= 0;
    
    for(size_t i=0;i<SIZE_+1;i++)
    {
        for(size_t j=0;j<SIZE_+1;j++)
        {
            if(i!=j)
            {
                mLaplacianMatrix(i, i)-=mLaplacianMatrix(i, j);
            }
        }
    }
    
    vector<int> NonZeros;
    for(size_t i=0;i<SIZE_+1;i++)
    {
        if (mLaplacianMatrix(i, i)>0.002)
        {
            NonZeros.push_back(i);
        }
    }
    int NonZerosSize=NonZeros.size();
    Eigen::MatrixXf NonZerosCol(SIZE_+1,NonZerosSize);
    for(size_t i=0;i<NonZeros.size();i++)
    {
        NonZerosCol.col(i)=mLaplacianMatrix.col(NonZeros[i]);
    }
    Eigen::MatrixXf NonZerosMatrix(NonZerosSize,NonZerosSize);
    for(size_t i=0;i<NonZeros.size();i++)
    {
        NonZerosMatrix.row(i)=NonZerosCol.row(NonZeros[i]);
    }
    Eigen::MatrixXf ReducedLaplacianMatrix;
    ReducedLaplacianMatrix.resize(NonZerosSize-1,NonZerosSize-1);
    ReducedLaplacianMatrix=NonZerosMatrix.block(0,0,NonZerosSize-1,NonZerosSize-1);
    
    //cerr<<"Size"<<ReducedLaplacianMatrix.size()<<endl;   
    float DOptimal= log(ReducedLaplacianMatrix.determinant());
    //cerr<<"Global Map"<<DOptimal<<endl;
    return DOptimal;
}



float Uncertainty::UpdateUncertaintyList(list<KeyFrame*> vpKFs,KeyFrame* pKF)
{
    int SIZE_=vpKFs.size();
    Eigen::MatrixXf mLaplacianMatrix;
    mLaplacianMatrix.resize(SIZE_+1,SIZE_+1);

    int i=0;
    for(list<KeyFrame*>::iterator lit=vpKFs.begin(),lend=vpKFs.end();lit!=lend;lit++)
    {
        int j=i-1;
        for(list<KeyFrame*>::iterator lit2=lit, lend2=vpKFs.end();lit2!=lend2;lit2++)
        {
            j++;
            if (lit!=lit2)
            {
                float temp=-((*lit)->GetWeight((*lit2)))*0.0002;
                mLaplacianMatrix(i, j)= temp;
                mLaplacianMatrix(j, i)= temp;
            }
            else
            {
                mLaplacianMatrix(j, i)= 0;
            }

        }
        float temp=-(*lit)->GetWeight(pKF)*0.0002;
        mLaplacianMatrix(i, SIZE_)= temp;
        mLaplacianMatrix(SIZE_, i)= temp;
        i++;
    }
    
    mLaplacianMatrix(SIZE_, SIZE_)= 0;
    for(size_t i=0;i<SIZE_+1;i++)
    {
        for(size_t j=0;j<SIZE_+1;j++)
        {
            if(i!=j)
            {
                mLaplacianMatrix(i, i)-=mLaplacianMatrix(i, j);
            }
        }
    }
    
    vector<int> NonZeros;
    for(size_t i=0;i<SIZE_+1;i++)
    {
        if (mLaplacianMatrix(i, i)>0.002)
        {
            NonZeros.push_back(i);
        }
    }
    int NonZerosSize=NonZeros.size();
    Eigen::MatrixXf NonZerosCol(SIZE_+1,NonZerosSize);
    for(size_t i=0;i<NonZeros.size();i++)
    {
        NonZerosCol.col(i)=mLaplacianMatrix.col(NonZeros[i]);
    }
    Eigen::MatrixXf NonZerosMatrix(NonZerosSize,NonZerosSize);
    for(size_t i=0;i<NonZeros.size();i++)
    {
        NonZerosMatrix.row(i)=NonZerosCol.row(NonZeros[i]);
    }
    Eigen::MatrixXf ReducedLaplacianMatrix;
    ReducedLaplacianMatrix.resize(NonZerosSize-1,NonZerosSize-1);
    ReducedLaplacianMatrix=NonZerosMatrix.block(0,0,NonZerosSize-1,NonZerosSize-1);
    
    //cerr<<"Size"<<ReducedLaplacianMatrix.size()<<endl;   
    float DOptimal= log(ReducedLaplacianMatrix.determinant());
    //cerr<<"Global Map"<<DOptimal<<endl;
    return DOptimal;
}

}

