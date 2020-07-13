#ifndef CIPDETECTORPARAM_H
#define CIPDETECTORPARAM_H

#include <string>

#include "CALSCParam.h"
#include "CSIFTParam.h"
#include "CProjParam.h"

class CIPDetectorParam{
public:
    CIPDetectorParam():m_nProcType(SIFT_ONLY){}
//    CIPParam(bool bSIFT){m_paramSIFT = CSIFTParam param(bSIFT);}

    string getProcessingType(){if (m_nProcType == SIFT_ONLY) return "SIFT";
                               else if (m_nProcType == SIFT_CSA) return "SIFT_CSA";
                               else if (m_nProcType == DOG_CSA) return "DOG_CSA";
                               else if (m_nProcType == SURF_CSA) return "SURF_CSA";
                               else if (m_nProcType == DOH_CSA) return "DOH_CSA";
                               else if (m_nProcType == SURF_ONLY) return "SURF";
                               else return "UNKNOWN";}

    int m_nProcType;    
    CSIFTParam m_paramSIFT;
    CALSCParam m_paramALSC;
    CProjParam m_paramProj;

    // processing type
    enum {SIFT_ONLY, SIFT_CSA, DOG_CSA, SURF_ONLY, SURF_CSA, DOH_CSA};
    // error code
    enum {NO_ERR, FILE_IO_ERR, SIP_DET_ERR, DESC_MATCH_ERR, CSA_MATCH_ERR};
};


#endif // CIPDETECTORPARAM_H
