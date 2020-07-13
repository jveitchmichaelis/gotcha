#ifndef CSURFPARAM_H
#define CSURFPARAM_H

class CSURFParam {
public :
    CSURFParam():m_nOctaves(4), m_nLayers(2), m_fContThr(500.f), m_fDescMatchThr(0.6f){}

    int m_nOctaves;
    int m_nLayers;
    float m_fContThr;
    float m_fDescMatchThr;
};

#endif // CSURFPARAM_H
