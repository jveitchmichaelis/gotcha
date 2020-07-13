#ifndef CRECTIFYPARAM_H
#define CRECTIFYPARAM_H

#include <string>
#include "CProjParam.h"
using namespace std;

class CRectifyParam{

public:
    CRectifyParam():m_nTPType(TP_UNKNOWN), m_nProcType(F_RANSAC), m_fMaxDis(1.5f), m_fConfLev(.99){}
    int m_nTPType;
    int m_nProcType;
    float m_fMaxDis;
    float m_fConfLev;

    string m_strInlierTPDir; // This is an optional parameter which store the filepath to save inlier TP

    CProjParam m_paramProj;

    string getTPType() {if (m_nTPType == TP_SIFT) return "TP_SIFT";
        else if (m_nTPType == TP_SIFT_CSA) return "TP_SIFT_CSA";
        else if (m_nTPType == TP_DOG_CSA) return "TP_DOG_CSA";
        else if (m_nTPType == TP_SURF) return "TP_SURF";
        else if (m_nTPType == TP_SURF_CSA) return "TP_SURF_CSA";
        else if (m_nTPType == TP_DOH_CSA) return "TP_DOH_CSA";
//        else if (m_nTPType == TP_DENSE) return "TP_DENSE"; // obsolete since ver 1.2
        else return "UNKNOWN";}
    string getProcessingType() {if (m_nProcType == F_7PT) return "7-point algorithm";
        else if (m_nProcType == F_8PT) return "8-point algorithm";
        else if (m_nProcType == F_RANSAC) return "RANSAC_8PtF";
        else if (m_nProcType == F_LMEDS) return "LMEDS";
        else return "UNKNOWN";}

    // processing type
    enum{F_7PT, F_8PT, F_RANSAC, F_LMEDS};
    // Type of TP used for processing
    enum{TP_UNKNOWN, TP_SIFT, TP_SIFT_CSA, TP_DOG_CSA, TP_SURF, TP_SURF_CSA, TP_DOH_CSA};//, TP_DENSE};
    // error code
    enum {NO_ERR, FILE_IO_ERR, F_ERR, RECTIFY_ERR};
};


#endif // CRECTIFYPARAM_H
