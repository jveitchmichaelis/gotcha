#ifndef CIPDETECTOR_H
#define CIPDETECTOR_H

#include "CIPDetectorParam.h"
#include "CTiePt.h"
#include "CIPDetectorParam.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <algorithm>
#include <numeric>
#include "ALSC.h"
#include "CInterestPoint.h"
#include "CAutoAdapt.h"
#include "CMutualAdapt.h"

// #include "CRefinedTP.h"

class CIPDetector{

public:
    CIPDetector();
    CIPDetector(CIPDetectorParam paramIP);

    void setParameters(CIPDetectorParam paramIP);    
    int performDetection(); // results will be saved in the m_strResDir and return the error code defined in CIPDetectorParam
    string getResDir() {return m_strResDirSub;}
    void setImages(const Mat &imgL,const Mat &imgR);
    void setOutputFile(string folder);
    string m_strExtEXEPath;         // need to know the path to the "resource"

private:
    bool saveSIFTParam(const CSIFTParam& paramSIFT, string strOut);


    bool getSIFTFeatures(bool bIsLeft);
    bool getSURFFeatures(bool bIsLeft);
    int isReadyToGo();
    void collectResult(CTiePt tp);

    bool saveTP(const string strFile, bool bCSA);
    bool saveTP(const vector<CTiePt>& vecTPs, const string strFile);
    bool saveTP(const vector<CTiePtAdapt>& vecTPsFromCSA, const string strFile);

    // descriptor matching functions
    bool matchDescriptors(bool bIsSURF = false);
    float getNearestNeighbour(const vector<float>::iterator iterDescL, const int nSizeDesc, int& nNei, float fMatchingThr, int nLIndex);
    float getNearestNeighbour(const vector<float>::iterator iterDescL, const int nSizeDesc, const int nClassID, int& nNei, float fMatchingThr, int nLIndex);
    float compareDescriptors(const vector<float>::iterator iterDescL, const vector<float>::iterator iterDescR, const int nSizeDesc, const float fSecondBestDist);
    bool parallelCompare(int tpid);
    float getMaxMatchingScore(vector<CTiePt>& points);
    Mat preprocessInputImg(bool bIsLeft); // make 1 channel and grey image

    // feature refine functions
//    bool getALSCmatching(bool bRefine);

    //
    bool getCSARefinement();
    bool getCSAMatching();
    bool saveLog(String strBase);
    bool saveResLog(string strFile);

    bool getDescriptor(int nPos, float* pfData, int nSzDesc, bool bLeft);



private:
    // inputs
    Mat m_imgL;
    Mat m_imgR;
    CIPDetectorParam m_paramIP;

    string m_strResDir;             // Result directory, e.g., .../(project_name)/Features
    string m_strResDirSub;
    string m_strOutputFile;

    // results
    vector<KeyPoint> m_vecKeyptsL; // you can save this result using a static function in KeyPoint class
    vector<KeyPoint> m_vecKeyptsR;
    vector <CTiePt> m_vecTPs;
    vector <CTiePtAdapt> m_vecTPsFromCSA;
    vector<float> m_vecDescL;      // exact size of a descriptor vector can be found by dividing by the size of the num of keypoints
    vector<float> m_vecDescR;
};

    static bool compareTps(const CTiePt point1, const CTiePt point2);

#endif // CIPDETECTOR_H
