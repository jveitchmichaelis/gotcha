#ifndef CAUTOADAPT_H
#define CAUTOADAPT_H

#include "CInterestPoint.h"

class CAutoAdapt
{
public:
    // add some more to deal with the 16-bit image case
    CAutoAdapt(Mat matImgSrc, const vector<KeyPoint>* pIPSrc) {m_matImgSrc = matImgSrc; m_bDeleteIP = false;
                                                               m_pvecIP = pIPSrc; m_pdAff = NULL;}
    CAutoAdapt(string strImg, string strIP); // this is useful for debugging
    ~CAutoAdapt();

    void doAutoAdaptation(const int nMaxIter = 15, const double dThrCicle = 0.05, const double dThrDist = 6);

    Mat getAffineParam(int nPos) const;
    double* getAllAffineParams() const {return m_pdAff;}
    bool isDataReady() const {if (m_pdAff != NULL) return true;
                              else return false;}
    bool saveAffineEstimation(string strFile) const;

protected:
    void clearOutputBuffer();
    Mat getHarrisRegionDesc(const int nIPPos, const Mat& matAff);
    float getSubPixelValue(const Point2f pt, const Mat& matImg);
    void getGradient(const Mat& matSrc, float* pfG, bool bIsGx);
    void getGaussianBlur(float* pfImgData, const Size szImg, Size szKernel, float fSigma);
    Mat getAffineNormalisingTR(const Mat& matRDesc);

private:
    // Inputs
    Mat m_matImgSrc;    
    const vector<KeyPoint>* m_pvecIP;
    bool m_bDeleteIP; // used only when the inputs are loaded from a file

    // Output
    double* m_pdAff;  // 4* m_pvecIP.size();
};

#endif // CAUTOADAPT_H
