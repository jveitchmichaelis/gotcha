#ifndef CALSCADAPT_H
#define CALSCADAPT_H

#include "ALSC.h"
class CALSCAdapt : public ALSC
{
public:
    CALSCAdapt();
    CALSCAdapt(Mat imgL, Mat imgR, CALSCParam paramALSC):ALSC(imgL, imgR, paramALSC){}
    CTiePt getRefinedTp(const int nPos){int nSz =getRefinedTps()->size();
                                        if (nPos < nSz && nPos >= 0) return ALSC::getRefinedTps()->at(nPos);
                                        else return CTiePt();}

    void performALSC(Point2f ptL, Point2f ptR);
    Mat getAffineTR(const int nPos, const int nALSPatchRad);
    Point2f getTrsnaformedPt(const int nPos, const Point2f ptCentre, const Point2f ptIn);

private:
    void saveMat(Mat& matIn, string strFile);// only for debugging
};

#endif // CALSCADAPT_H
