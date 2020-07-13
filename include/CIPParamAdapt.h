#ifndef CIPPARAMADAPT_H
#define CIPPARAMADAPT_H

class CIPParamAdapt
{
public:
    CIPParamAdapt();
    CIPParamAdapt(int nIPType);

    void setSIFTParam();
    void setSURFParam();

    enum IP_TYPE{IP_UNKNOWN, IP_DOG, IP_DOH};

    int m_nType;
    int m_nOctaves;
    int m_nLayers;
    float m_fContThr;
    int m_nStartingOct;
    float m_fEdgeThr;
};

#endif // CIPPARAMADAPT_H
