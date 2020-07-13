#ifndef CMUTUALADAPT_H
#define CMUTUALADAPT_H

#include "CInterestPoint.h"
#include "CAutoAdapt.h"
#include "CTiePtAdapt.h"

class CMutualAdapt
{
public:
    CMutualAdapt(string strImgL, string strImgR, string strIPL, string strIPR, string strAutoShapeL, string strAutoShapeR);
    CMutualAdapt(Mat matImgL, Mat matImgR, CInterestPoint* pIpL, CInterestPoint* pIpR,
                 double* pdAutoShapeParamL, double* pdAutoShapeParamR) {
        m_matImgL = matImgL; m_matImgR = matImgR;
        m_pIpL = pIpL; m_pIpR = pIpR;
        m_pdAutoShapeParamL = pdAutoShapeParamL;
        m_pdAutoShapeParamR = pdAutoShapeParamR;
        m_bDeletePointer = false;
        m_fDescMatchingThr = 0.6f;
    }
    ~CMutualAdapt();

    void doMutualAdaptation();
    void doMutualAdaptationForRefinement();
    void setMatchingThr(float fDescMatchThr) {m_fDescMatchingThr = fDescMatchThr;}


    bool saveTPResult(string strFile, bool bDistortionParam = true) const;
    vector<CTiePtAdapt> getResult() const {return m_vecTP;}

protected:
    bool loadAutoShape(string strFile, bool bIsLeft = true); // for debugging

    int getMatchingCandidates(const int nPosL);
    float compareDescriptors(const int nPosL, const int nPosR, const vector<float>* pvecDescR, const float fSecondBestDist);

    bool validateMatch(const int nPosL, const int nPosR, CTiePtAdapt& tp);
    Mat getNormalisedPatch(const int nPos, const int nPosR, const int nALSCPatchRad, double* pdInvNorm, bool bIsLeft);
    Mat cropImage(const int nPos, const int nALSCPatchRad, bool isLeft);
    float getSubPixelValue(const Point2f pt, const Mat& matImg);

    void collectTP(CTiePtAdapt& tpIn);
    void makeOneToOne();

private:
    void saveMat(Mat& matIn, string strFile);// only for debugging

private:
    // inputs
    Mat m_matImgL; // image
    Mat m_matImgR;
    CInterestPoint* m_pIpL; // ip and descriptor
    CInterestPoint* m_pIpR;
    double* m_pdAutoShapeParamL; // this is transfrom that change a circle to an ellipse
    double* m_pdAutoShapeParamR; // four affine parameters, e.g., [a11 a12; a21 a22]
    bool m_bDeletePointer;
    float m_fDescMatchingThr;

    //outputs
    vector<CTiePtAdapt> m_vecTP;    
};

#endif // CMUTUALADAPT_H
