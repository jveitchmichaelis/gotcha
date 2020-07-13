#ifndef CALSCPARAM_H
#define CALSCPARAM_H

class CALSCParam {

public:
    CALSCParam():m_nMaxIter(10),m_nPatch(12),m_fEigThr(30.f),m_fAffThr(2.f),m_fDriftThr(1.5f),
                 m_bWeighting(false),m_bIntOffset(true)/*,m_nMaxSearchDist(-1)*/{}

    // matching param
    int m_nMaxIter; // the max num of iterations
    int m_nPatch;   // the size of a matching patch
    float m_fEigThr;
    float m_fAffThr;
    float m_fDriftThr;
    bool m_bWeighting;
    bool m_bIntOffset;

////   ver. 1.0
////   changed to offset!
////   optional parameter to limit matching candidates used only in ALSC_LOG or ALSC_DOH
    // since ver. 1.1
    // find matching candidates from
    // < 0 : whole image
    // = 0 : image strip ( 0, l_y-2, the width of the right image, 4)
    // > 0 : ( l_x-m_nMaxSearchDist, l_y-m_nMaxSearchDist, m_nMaxSearchDist*2, m_nMaxSearchDist*2)
//    int m_nMaxSearchDist;
//    bool m_bIsUsed; // this is an indicator used only in GOTCHA
};

#endif // CALSCPARAM_H

