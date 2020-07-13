#ifndef CDENSIFYPARAM_H
#define CDENSIFYPARAM_H

#include <string>

#include "CProjParam.h"
#include "CGOTCHAParam.h"

using namespace std;

class CDensifyParam {

public:
    CDensifyParam(): m_nTPType(TP_UNKNOWN), m_nProcType(GOTCHA){}
    int m_nTPType;       // type of input points
    int m_nProcType;     // growing method
    bool m_bUseInlierTP;


    string getProcessingType(){ if (m_nProcType == GOTCHA) return "GOTCHA";
                                else if (m_nProcType == P_GOTCHA) return "P_GOTCHA";
                                else return "UNKNOWN";}
    string getTPType() {if (m_nTPType == TP_SIFT && !m_bUseInlierTP) return "TP_SIFT";
                        else if (m_nTPType == TP_SIFT_CSA && !m_bUseInlierTP) return "TP_SIFT_CSA";
                        else if (m_nTPType == TP_DOG_CSA && !m_bUseInlierTP) return "TP_DOG_CSA";
                        else if (m_nTPType == TP_SURF && !m_bUseInlierTP) return "TP_SURF";
                        else if (m_nTPType == TP_SURF_CSA && !m_bUseInlierTP) return "TP_SURF_CSA";
                        else if (m_nTPType == TP_DOH_CSA && !m_bUseInlierTP) return "TP_DOH_CSA";
                        else if (m_nTPType == TP_SIFT && m_bUseInlierTP) return "TP_SIFT (inlier only)";
                        else if (m_nTPType == TP_SIFT_CSA && m_bUseInlierTP) return "TP_SIFT_CSA (inlier only)";
                        else if (m_nTPType == TP_DOG_CSA && m_bUseInlierTP) return "TP_DOG_CSA (inlier only)";
                        else if (m_nTPType == TP_SURF && m_bUseInlierTP) return "TP_SURF (inlier only)";
                        else if (m_nTPType == TP_SURF_CSA && m_bUseInlierTP) return "TP_SURF_CSA (inlier only)";
                        else if (m_nTPType == TP_DOH_CSA && m_bUseInlierTP) return "TP_DOH_CSA (inlier only)";
                        else if (m_nTPType == TP_DENSE) return "TP from the previous densification result";
                        else if (m_nTPType == TP_RECTIFIED) return "TP from the rectification result";
                        else return "UNKNOWN";}

    CProjParam m_paramProj;
    CGOTCHAParam m_paramGotcha;
//    CDPParam m_paramDP;

////    DPParam m_paramDP;
////    LineGOTCHAParam m_paramLineGotcha;
    enum{TP_UNKNOWN, TP_SIFT, TP_SIFT_CSA, TP_DOG_CSA, TP_SURF, TP_SURF_CSA, TP_DOH_CSA, TP_DENSE, TP_RECTIFIED};
    enum{GOTCHA, P_GOTCHA};
    enum{NO_ERR, FILE_IO_ERR, GOTCHA_ERR, LINE_GOTCHA_ERR, P_GOTCHA_ERR};
};
#endif // CDENSIFYPARAM_H
