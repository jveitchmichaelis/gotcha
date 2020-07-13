#include "CIPParamAdapt.h"

CIPParamAdapt::CIPParamAdapt()
{
    setSIFTParam();
}

CIPParamAdapt::CIPParamAdapt(int nIPType){
    if (nIPType == IP_DOG)
        setSIFTParam();
    else if (nIPType == IP_DOH)
        setSURFParam();
    else {// if (nType == IP_UNKNOWN)
        setSIFTParam();
    }
}

void CIPParamAdapt::setSIFTParam(){
    m_nType = IP_DOG;
    m_nOctaves = 4;
    m_nLayers = 3;
    m_nStartingOct = 0;
    m_fEdgeThr = 10.f;
    m_fContThr = 0.005f; // contrast threshold
}

void CIPParamAdapt::setSURFParam(){
    m_nType = IP_DOH;
    m_nOctaves = 4;     // default 4
    m_nLayers = 2;      // default 2
    m_nStartingOct = 0; // not used in SURF
    m_fEdgeThr = 0.f;   // not used in SURF
    m_fContThr = 500.f;   // Threshold for the Determinant of Hessain in SURF IP detector: default 500 (300-500)
}
