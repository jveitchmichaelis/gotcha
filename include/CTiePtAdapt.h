#ifndef CTIEPTADAPT_H
#define CTIEPTADAPT_H

#include "CTiePt.h"
class CTiePtAdapt: public CTiePt
{    
    // When CTiePt is used in ALSC
    // m_pfAffine defines [da11 da12 da21 da22] and centre ofsset, m_ptOffset
    // In CMutualAdapt
    // m_pfAffine and m_pfAffineL are used to store Affine params from auto shape adaptation
    // m_ptOffset and m_ptOffsetL are offset from the centre
    // m_matALSCAff is affine transfrom

public:
    CTiePtAdapt():CTiePt(){
        for (int i = 0; i < 4; i++)
            m_pfAffineL[i] = 0.f;
        m_pfAffineL[0] = 1.f;
        m_pfAffineL[3] = 1.f;
        m_pfAffine[0] = 1.f;
        m_pfAffine[3] = 1.f;
        m_ptOffsetL = Point2f(0,0);

        m_matALSCAff = Mat::zeros(3,3,CV_64FC1);
        m_matNormL = Mat::zeros(3,3,CV_64FC1);
        m_matNormR = Mat::zeros(3,3,CV_64FC1);
//        m_matAff_LR = Mat::zeros(3,3,CV_64FC1);
        m_matALSCAffL =  Mat::zeros(3,3,CV_64FC1);
//        m_bALSCAffReady = false;
    }

    Point2f m_ptOffsetL;
    float m_pfAffineL[4];

//    bool operator==(const CTiePt& x){
//        if ((this->m_ptL == x.m_ptL) || (this->m_ptR == x.m_ptR))
//            return true;
//        else return false;
//     }

    Mat getSR(bool bLeft){
        Mat matSR;
        Mat matNorm;
        Mat matAff = Mat::zeros(3,3, CV_64FC1); matAff.at<double>(2,2) = 1;
        if (bLeft) { matNorm = m_matNormL;
                     matAff.at<double>(0,0) = m_pfAffineL[0]; matAff.at<double>(0,1) = m_pfAffineL[1];
                     matAff.at<double>(1,0) = m_pfAffineL[2]; matAff.at<double>(1,0) = m_pfAffineL[3];}
        else { matNorm = m_matNormR;
               matAff.at<double>(0,0) = m_pfAffine[0]; matAff.at<double>(0,1) = m_pfAffine[1];
               matAff.at<double>(1,0) = m_pfAffine[2]; matAff.at<double>(1,0) = m_pfAffine[3];}
        //matNorm = matSR*matAff.inv();
        matSR = matNorm*matAff;
        return matSR;
    }

    Mat getDelAff(bool bLeft){
        Mat matSR = getSR(bLeft);
        Mat matALSC;
        if (bLeft) matALSC = m_matALSCAffL;
        else matALSC = m_matALSCAff;
        // (matALSCAff*matSR)*matAff^(-1) * X_l = (matALSCAff_r*matSR_r)*matAff_r^(-1) * X_r
        Mat matDelAff = matALSC*matSR;
        return matDelAff;
    }

    ////////////////////////////////////////////////////////////////////
    // following parameters are used in Mutual adaptation
    // only used when m_bALSCAffReady == true
    ////////////////////////////////////////////////////////////////////
//    bool m_bALSCAffReady;
    Mat m_matALSCAff; // 3-by-3 affine transform between original boundary and ALSC-updated boundary in a right normalised images
    Mat m_matALSCAffL;
    Mat m_matNormL;   // 3-by-3 double type, S*R*A(m_pfAffine)^-1
    Mat m_matNormR;   // 3-by-3
//    Mat m_matAff_LR;  // 3-by-3 affine transform between tiepoint in a normalised plane
};
#endif // CTIEPTADAPT_H
