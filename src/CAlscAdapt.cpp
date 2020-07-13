#include "CAlscAdapt.h"
#include <iostream>
#include <fstream>

CALSCAdapt::CALSCAdapt()
{
}

void CALSCAdapt::performALSC(Point2f ptL, Point2f ptR){
    vector<CTiePt> vecTp;
    CTiePt tpIn;
    tpIn.m_ptL = ptL;
    tpIn.m_ptR = ptR; // 1
    vecTp.push_back(tpIn); // input

    ALSC::performALSC(&vecTp);    
}

Mat CALSCAdapt::getAffineTR(const int nPos, const int nALSPatchRad){
    // As ALSC only estimates dA and dt
    // a full affine transfrom is estimated here

    Mat matRes = Mat::zeros(2,2, CV_64FC1);
    matRes.at<double>(1,1) = 1;

    // collect from the boundary points
    Point2f ptSrc[3];
    Point2f ptDes[3];

    CTiePt tp = getRefinedTp(nPos);
    Point2f ptCentre = tp.m_ptR - tp.m_ptOffset;

    ptSrc[0] = Point2f(ptCentre.x-nALSPatchRad, ptCentre.y-nALSPatchRad);
    ptSrc[1] = Point2f(ptCentre.x-nALSPatchRad, ptCentre.y+nALSPatchRad);
    ptSrc[2] = Point2f(ptCentre.x+nALSPatchRad, ptCentre.y-nALSPatchRad); // initial boundary

    for (int i = 0 ; i < 3; i++)
        ptDes[i] = getTrsnaformedPt(nPos, ptCentre, ptSrc[i]);

    // get affine transform from
    Mat matTemp = getAffineTransform(ptSrc, ptDes);

    for (int j = 0; j < 2; j++){
        for (int i = 0; i < 2; i++){
            matRes.at<double>(j, i) = matTemp.at<double>(j,i);            
        }
    }

//    saveMat(matTemp,"/Users/DShin/Desktop/a.txt");

    return matRes;
}


Point2f CALSCAdapt::getTrsnaformedPt(const int nPos, const Point2f ptCentre, const Point2f ptIn){
    Point2f ptRes;

    CTiePt tp = getRefinedTp(nPos);

    double dOffsetX = 0, dOffsetY = 0;
    double dx = ptIn.x - ptCentre.x;
    double dy = ptIn.y - ptCentre.y;

    dOffsetX = dx * tp.m_pfAffine[0] + dy * tp.m_pfAffine[1];
    dOffsetY = dx * tp.m_pfAffine[2] + dy * tp.m_pfAffine[3];
    ptRes.x = ptIn.x + dOffsetX;
    ptRes.y = ptIn.y + dOffsetY;

    return ptRes;
}

void CALSCAdapt::saveMat(Mat& matIn, string strFile){
    ofstream sfTC;
    sfTC.open(strFile.c_str());

    if (sfTC.is_open()){
        for (int i = 0; i < matIn.rows; i++){
            for (int j = 0 ; j < matIn.cols; j++){
                if (matIn.depth() == CV_64F)
                    sfTC << matIn.at<double>(i,j) << " ";
                else if (matIn.depth() == CV_32F)
                    sfTC << matIn.at<float>(i,j) << " ";
            }
            sfTC << endl;
        }
        sfTC.close();
    }
}
