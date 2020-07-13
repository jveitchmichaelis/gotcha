#ifndef CRECTIFY_H
#define CRECTIFY_H

#include "CProcBlock.h"
#include "CRectifyParam.h"

#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

class CRectify : public CProcBlock
{

public:
    CRectify();
    CRectify(CRectifyParam paramRect);

    int performRectification();

    void setParameters(CRectifyParam paramRect);
    int getNumInliers() const {return m_vecnInliers.size();}
    float getFestimationErr() const {return m_fFerr;}    

private:
    void setResultDir(string strProjDir);
    bool saveResult();

    bool estimateF();
    Mat get8ptF();
    Mat get8ptF(const vector<int>::iterator itrIdxBegin, const int nSize);
//    Mat get8ptF(const vector<Point2f>::iterator itrPtLBegin, const vector<Point2f>::iterator itrPtLEnd, const int nPtLSz, const int nPtRSz);
    Mat getRANSACF(float fConf, float fDistThr);
    void getRandomIndx(int nMin, int nMax, vector<int>& vecRndIdx);
//    Mat getModel(const vector<int>::iterator iter, const int nSize);
    void getConsensusSet(const Mat matM, const float fDistThr, vector<int>& vecnConIdx);
    float getModelFittingErr(Mat matMbetter, const vector<int>::iterator iter, const int nSize);

    bool getRectifyTransforms();
    bool estimateHR();
    bool estimateHL();
    void getNewTr(bool bIsLeft);
    bool isLeft(const Mat matA, const Mat matB, const Mat matC);

    bool saveLog();
    bool saveResLog(string strFile);
    bool makeDisparityMaps(string strOutDir); // not used since ver.1.2

private:
    // inputs    
    CRectifyParam m_paramRect;
    string m_strResDir; // output dir:  .../<proj>/Rectify

    // temp variables : subset of tiepoints used for F estimation
//    vector<Point2f> m_vecPtL;
//    vector<Point2f> m_vecPtR;

    // outputs
    Mat m_matF;                  // from left to rihgt
    Mat m_matHL;                 // rectifying transform for a left image
    Mat m_matHR;    
    Mat m_matRectImgL;           // rectified image
    Mat m_matRectImgR;        
    vector<int> m_vecnInliers;   // index of inlier TPs
    float m_fFerr;
};

#endif // CRECTIFY_H
