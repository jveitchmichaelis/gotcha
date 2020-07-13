#ifndef CINTERESTPOINT_H
#define CINTERESTPOINT_H

#include <iostream>
#include <fstream>
#include <math.h>

#include <opencv2/opencv.hpp>

#include "CIPParamAdapt.h"
#include "CTiePt.h"

using namespace cv;
using namespace std;

// This class requires SIFT executable and associated library from UCLA
// Please install
class CInterestPoint
{
public:
    CInterestPoint(); // Default constructor which may require for the class inheritence later

    // By default, assume "strRscPath" is not required, i.e., "sift" is included in the main environment path for the executables.
    // Please note that the processing parameters ('pparam') are passed as a pointer type, which makes the paramers
    // of these functions always synchronised with the ones defined outside of the class.
    // NB. OpenCV Mat is a reference type
    CInterestPoint(Mat matImg, CIPParamAdapt* pparam);
    CInterestPoint(Mat matImg, CIPParamAdapt* pparam, string strRscPath);
//    CInterestPoint(Mat matImg, string strAllRes); // this is helpful for debugging
//    ~CInterestPoint(){if (bDelete) delete [] m_pparamIP;}// this is helpful for debugging

    // setters
    void setInputImage(Mat matImg) {m_matImgSrc = matImg;}
    void setProcParam(CIPParamAdapt* pparamIP) {m_pparamIP = pparamIP;}
    void setResourceDir(string strRscPath) {m_strRscPath = strRscPath;}
    bool setResult(string strAllRes); // this function sounds bizzare but helpful for debugging CMutualAdapt.
                                      // It just uses class as header. Don't use unless you know what to do
	void addIP(KeyPoint kp, float * pfDesc, int nSzDesc) {
		m_vecIPs.push_back(kp);
		for (int i = 0; i < nSzDesc; i++)
			m_vecDesc.push_back(pfDesc[i]);
	}

    //bool updateAffineParam (const int nPos, const Mat& matAff, bool bIsLeft);

    // Image interesting point would be estimated from
    // Differece of Gaussian (DoG used in SIFT)
    // or Approx. Determinat of Hessian (DoH used in SURF)
    bool detectIPs();
    void clearResultBuffers() {m_vecIPs.empty(); m_vecDesc.empty();}

    // get processed data
    const vector<KeyPoint>* getIPs() {return &m_vecIPs;} // the processed data cannot be changed outside
    const vector<float>* getDescriptor() {return &m_vecDesc;}
    vector<KeyPoint> getClonedIPs() const;                     // a hard copy of the IP detection result
    vector<float> getClonedDescriptors() const;
    int getTypeOfIP() {return m_pparamIP->m_nType;}
    int getDescriptorLength() const {if (m_vecIPs.size() == 0) return 0;
                                     else return (int)m_vecDesc.size()/m_vecIPs.size();}
    int getNumOfIPs() {return m_vecIPs.size();}

    // save results
    bool saveIPRes(string strPath);
    bool saveDescRes(string strPath);
    bool saveAllResults(string strPath); // save ip and descriptor

    // matching functions
    vector<CTiePt> getTCfromDescMatching(CInterestPoint& ipRight);
    bool saveTPResult(const vector<CTiePt>& vecTC, string strFile, bool bDistortionParam = false) const;

 protected:
    bool getSIFTFeatures();
    Mat preprocessInputImg();  // convet to a grey image
    bool isFileExists(string strFile);

    // matching
    float getMatchingDesc(const int nPos, const vector<float>* pvecDescR, int& nNei); // for SIFT matching
    float getMatchingDesc(const int nPos, const vector<float>* pvecDescR, const vector<KeyPoint>* pvecIPR, const int nClassID, int& nNei); // for SURF matching
    float compareDescriptors(const int nPosL, const int nPosR, const vector<float>* pvecDescR, const float fSecondBestDist);

private:
    // inputs
    const CIPParamAdapt* m_pparamIP;       // Pointer to the processing parameters. Do not change the values of m_paramIP
    Mat m_matImgSrc;           // this is a soft copy (i.e., reference to) of the input image
    string m_strRscPath;       // directory where the SIFT executable and the associated library are
//    bool bDelete;

    // outputs
    vector<KeyPoint> m_vecIPs; // you can save this result
    vector<float> m_vecDesc;   // the size of a single descriptor vector can be defiend by dividing
                               // the size of m_vecDesc by the size of the number of keypoints
};

#endif // CINTERESTPOINT_H
