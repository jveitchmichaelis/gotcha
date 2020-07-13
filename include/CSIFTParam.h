#ifndef CSIFTPARAM_H
#define CSIFTPARAM_H

#include "CSURFParam.h"
class CSIFTParam : public CSURFParam{
public:
//    CSIFTParam(): m_nOctaves(4), m_nLayers(3), m_nStartingOct(0), m_fEdgeThr(10.f), m_fContThr(0.005f), m_fDescMatchThr(0.6f){}
//    setForSIFT(){m_nOctaves=4; m_nLayers=3; m_nStartingOct=0; m_fEdgeThr=10.f; m_fContThr=0.005f;}
//    setForSURF(){m_nOctaves=4; m_nLayers=2; m_fContThr=500.f; /*not used ->*/ m_nStartingOct=0; m_fEdgeThr=0.f;}    
    CSIFTParam(){m_nLayers = 3;
                 m_nStartingOct = 0;
                 m_fEdgeThr = 10.f;
                 m_fContThr = 0.005f;}

    int m_nStartingOct;
    float m_fEdgeThr;    
};

#endif // CSIFTPARAM_H
