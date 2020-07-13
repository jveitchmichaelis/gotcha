#include "CIPDetector.h"

#define PI 3.141592653589793

using namespace std;

CIPDetector::CIPDetector()
{
}

CIPDetector::CIPDetector(CIPDetectorParam paramIP){
    m_paramIP = paramIP;
    m_strResDir = "./";
}

void CIPDetector::setImages(const Mat &imgL,const Mat &imgR){
    m_imgL = imgL;
    m_imgR = imgR;
}

void CIPDetector::setOutputFile(string file){
    m_strOutputFile = file;
}

int CIPDetector::isReadyToGo(){
    // images are ready?
    if (m_imgL.data == NULL || m_imgR.data == NULL) return CIPDetectorParam::FILE_IO_ERR;
    return CIPDetectorParam::NO_ERR;
}

bool CIPDetector::saveTP(const string strFile, bool bCSA){
    if (!bCSA)
        return saveTP(m_vecTPs, strFile);
    else
        return saveTP(m_vecTPsFromCSA, strFile);
}

bool CIPDetector::saveTP(const vector<CTiePtAdapt>& vecTPsFromCSA, const string strFile){
    ofstream sfTP;
    // string strOut = strFile; //  m_strProj + m_paramIP.m_paramProj.m_strSep + strTPFile;
//    if (!bAppend)
    sfTP.open(strFile.c_str());
//    else
//        sfTP.open(strFile.c_str(), ios::app | ios::out);

    int nLen = vecTPsFromCSA.size();
    int nEle = 25;
//    if (nLen <= 0)  {
//        sfTP.close();
//        return false;
//    }

    if (sfTP.is_open()){
        // header
        sfTP << nLen << " " << nEle << endl;
        // data
        for (int i = 0 ; i < nLen ;i++){
            CTiePtAdapt tp = vecTPsFromCSA.at(i);
            sfTP << tp.m_ptL.x + tp.m_ptOffsetL.x << " " << tp.m_ptL.y + tp.m_ptOffsetL.y << " "
                 << tp.m_ptR.x + tp.m_ptOffset.x << " " << tp.m_ptR.y + tp.m_ptOffset.y  << " " << tp.m_fSimVal << " "
                 // left auto shape distortion
                 << tp.m_pfAffineL[0] << " " << tp.m_pfAffineL[1] << " "
                 << tp.m_pfAffineL[2] << " " << tp.m_pfAffineL[3] << " "
                 << tp.m_ptOffsetL.x << " " << tp.m_ptOffsetL.y << " "
                 // rightt auto shape distortion
                 << tp.m_pfAffine[0] << " " << tp.m_pfAffine[1] << " "
                 << tp.m_pfAffine[2] << " " << tp.m_pfAffine[3] << " "
                 << tp.m_ptOffset.x << " " << tp.m_ptOffset.y ;

            // mutual sahpe distortion
            for (int m = 0 ; m < 2; m++){
                for (int n = 0; n < 2; n++){
                    sfTP << " " << tp.m_matALSCAffL.at<double>(m,n);
                }
            }

            for (int m = 0 ; m < 2; m++){
                for (int n = 0; n < 2; n++){
                    sfTP << " " << tp.m_matALSCAff.at<double>(m,n);
                }
            }
            sfTP << endl;
        }
        sfTP.close();
    }
    else
        return false;

    return true;
}

bool CIPDetector::saveTP(const vector<CTiePt>& vecTPs, const string strFile){

    ofstream sfTP;

    std::cout << strFile << std::endl;

    sfTP.open(strFile.c_str());
    int nLen = vecTPs.size();
    int nEle = 11;

    if (sfTP.is_open()){
        // header
        sfTP << nLen << " " << nEle << endl;
        // data
        for (int i = 0 ; i < nLen ;i++){
            CTiePt tp = vecTPs.at(i);
            sfTP << tp.m_ptL.x << " "<< tp.m_ptL.y << " "
                 << tp.m_ptR.x << " "<< tp.m_ptR.y << " "
                 << tp.m_fSimVal << " "<< tp.m_pfAffine[0] << " "
                 << tp.m_pfAffine[1] << " "<< tp.m_pfAffine[2] << " "
                 << tp.m_pfAffine[3] << " " << tp.m_ptOffset.x << " "
                 << tp.m_ptOffset.y << endl;

        }
        sfTP.close();
    }
    else
        return false;

    return true;
}

int CIPDetector::performDetection(){

    // validity checking
    int nRes = isReadyToGo();
    if (nRes != CIPDetectorParam::NO_ERR) return nRes;

    // define result saving directory
    // clear previous results
    m_vecDescL.clear();
    m_vecKeyptsL.clear();
    m_vecDescR.clear();
    m_vecKeyptsR.clear();

    m_vecTPs.clear(); // it is sensible to clear tps at this point!

    string strTPFile;

    // preparation of features
    //cout << "Started feature detection at: " << QTime::currentTime().toString("HH:mm:ss").toStdString() << endl;
    if (!getSIFTFeatures(true) || !getSIFTFeatures(false))
        return CIPDetectorParam::SIP_DET_ERR;

    // descriptor matching
    if (m_paramIP.m_nProcType == CIPDetectorParam::SIFT_ONLY ||
        m_paramIP.m_nProcType == CIPDetectorParam::SIFT_CSA )
    {
        if (!matchDescriptors()) return CIPDetectorParam::DESC_MATCH_ERR;
        strTPFile = FILE_TP_SIFT;
    }

    if (m_paramIP.m_nProcType == CIPDetectorParam::SIFT_CSA){

        if (!getCSARefinement()) return CIPDetectorParam::CSA_MATCH_ERR;
        strTPFile = FILE_TP_SIFT_CSA;
    }

    // save results
    //QString strTime = QDate::currentDate().toString("[DyyMMdd_T")+QTime::currentTime().toString("hhmmss]");

    // create folder
    bool bCSA = false;
    if (m_paramIP.m_nProcType != CIPDetectorParam::SURF_ONLY &&
        m_paramIP.m_nProcType != CIPDetectorParam::SIFT_ONLY) bCSA = true;
    if (!saveTP(m_strOutputFile, bCSA))
        return CIPDetectorParam::FILE_IO_ERR;

    return CIPDetectorParam::NO_ERR;
}

bool CIPDetector::getDescriptor(int nPos, float* pfData, int nSzDesc, bool bLeft){
    if (pfData == NULL) return false;

    vector<float>* m_vecDesc;
    if (bLeft)
        m_vecDesc = &m_vecDescL;
    else
        m_vecDesc = &m_vecDescR;

    /* RootSIFT */

    float desc_sum;
    for (auto&i : *m_vecDesc)
        desc_sum += i;

    for (auto& i : *m_vecDesc){
        i /= desc_sum;
        i = sqrt(i);
    }

    for (int i = 0 ; i < nSzDesc; i++){
        pfData[i] = m_vecDesc->at(nSzDesc*nPos+i);
    }

    return true;
}

bool CIPDetector::getCSARefinement(){

    this->m_vecTPsFromCSA.empty();

    ///////////////////////////////
    // DATA selection
    CInterestPoint ipL; ipL.setInputImage(m_imgL);
    CInterestPoint ipR; ipR.setInputImage(m_imgR);

    int nSzTP = m_vecTPs.size();
    int nSzDesc = m_vecDescL.size() / m_vecKeyptsL.size();
    float* pfData = new float [nSzDesc];

    for (int i = 0 ; i < nSzTP; i++){
        CTiePt tp = m_vecTPs.at(i);

        KeyPoint kpL = m_vecKeyptsL.at(tp.m_ptIDPair.x);
        getDescriptor(tp.m_ptIDPair.x, pfData, nSzDesc, true);
        ipL.addIP(kpL, pfData , nSzDesc);

        KeyPoint kpR = m_vecKeyptsR.at(tp.m_ptIDPair.y);
        getDescriptor(tp.m_ptIDPair.y, pfData, nSzDesc, false);
        ipR.addIP(kpR, pfData, nSzDesc);
    }

    ///////////////////////////////
    // 1. auto shape update from the descriptor matchings
    cout << "Starting Auto shape adaptation" << endl;
    CAutoAdapt autoShL(m_imgL, ipL.getIPs());
    CAutoAdapt autoShR(m_imgR, ipR.getIPs());
    autoShL.doAutoAdaptation();
    autoShR.doAutoAdaptation();

    // 2. mutual shape update from 1.
    cout << "Starting Mutual shape adaptation" << endl;
    CMutualAdapt muShape(m_imgL, m_imgR, &ipL, &ipR, autoShL.getAllAffineParams(), autoShR.getAllAffineParams());
    muShape.doMutualAdaptationForRefinement();

    // 3. collect rsults in m_vecTPsFromCSA
    m_vecTPsFromCSA  = muShape.getResult();
    cout << "Completed CSA refinement at:" << endl;
    return true;
}

bool CIPDetector::getCSAMatching(){
    CInterestPoint ipL; ipL.setInputImage(m_imgL);
    CInterestPoint ipR; ipR.setInputImage(m_imgR);

    int nSzKP = m_vecKeyptsL.size();
    int nSzDesc = m_vecDescL.size() / m_vecKeyptsL.size();
    float* pfData = new float [nSzDesc];

    for (int i = 0 ; i < nSzKP; i++){
        KeyPoint kpL = m_vecKeyptsL.at(i);
        ipL.addIP(kpL, pfData , nSzDesc);
        getDescriptor(i, pfData, nSzDesc, true);
    }

    nSzKP = m_vecKeyptsR.size();
    if ( nSzDesc != (int) (m_vecDescR.size() / m_vecKeyptsR.size())) return false;
    for (int i = 0 ; i < nSzKP; i++){
        KeyPoint kpR = m_vecKeyptsR.at(i);
        ipR.addIP(kpR, pfData , nSzDesc);
        getDescriptor(i, pfData, nSzDesc, false);
    }

    ///////////////////////////////
    // 1. auto shape update from the descriptor matchings
    cout << "Starting Auto shape adaptation" << endl;
    CAutoAdapt autoShL(m_imgL, ipL.getIPs());
    CAutoAdapt autoShR(m_imgR, ipR.getIPs());
    autoShL.doAutoAdaptation();
    autoShR.doAutoAdaptation();

    // 2. mutual shape update from 1.
    cout << "Starting Mutual shape adaptation" << endl;
    CMutualAdapt muShape(m_imgL, m_imgR, &ipL, &ipR, autoShL.getAllAffineParams(), autoShR.getAllAffineParams());
    muShape.setMatchingThr(m_paramIP.m_paramSIFT.m_fDescMatchThr * 0.8);
    muShape.doMutualAdaptation();

    // 3. collect rsults in m_vecTPsFromCSA
    m_vecTPsFromCSA  = muShape.getResult();
    cout << "Completed CSA refinement" << endl;
    return true;
}


bool CIPDetector::saveLog(String strBase){

//    string strFile = m_strResDir + m_paramIP.m_paramProj.m_strSep + FILE_LOG;
    string strFile = strBase + m_paramIP.m_paramProj.m_strSep + FILE_LOG;
    bool bRes = false;
    // save parameter log
    bRes = saveSIFTParam(m_paramIP.m_paramSIFT, strFile);
//    bRes = bRes && saveALSCParam(m_paramIP.m_paramALSC, strFile);
    // save result log
    bRes = bRes && saveResLog(strFile);
    return bRes;
}

bool CIPDetector::saveSIFTParam(const CSIFTParam& paramSIFT, string strOut){
    ofstream sfLog;
    sfLog.open(strOut.c_str(), ios::app | ios::out);

    if (sfLog.is_open()){

        sfLog << "<SIFT/SURF Parameters>" << endl;
        sfLog << "Total number of octaves in a scale space: " << paramSIFT.m_nOctaves << endl;
        sfLog << "Total number of layers within an octave: " << paramSIFT.m_nLayers << endl;
        sfLog << "Starting octave ID (SIFT only): " << paramSIFT.m_nStartingOct << endl;
        sfLog << "DOG/DOH threshold: " << paramSIFT.m_fContThr << endl;
        sfLog << "Edge threshold (SIFT only): " << paramSIFT.m_fEdgeThr << endl;
        sfLog << "Descriptor matching threshold: " << paramSIFT.m_fDescMatchThr << endl;
        sfLog << endl;

        sfLog.close();
    }
    else
        return false;

    return true;
}

bool CIPDetector::saveResLog(string strFile){
    bool bRes = false;

    ofstream sfLog;
    sfLog.open(strFile.c_str(), ios::app | ios::out);

    if (sfLog.is_open()){
        sfLog << "<Processing results>" << endl;
        sfLog << "Processing method: " << m_paramIP.getProcessingType() << endl;
        sfLog << "Total number of IPs (L/R): " << m_vecKeyptsL.size() << "/" << m_vecKeyptsR.size() << endl;

        sfLog.close();
        bRes = true;
    }
    else
        return bRes = false;

    return bRes;
}

bool CIPDetector::parallelCompare(int tpid){
    int nNei = -1;
    float fMatchingScore = -1.f;

    vector<float>::iterator iterDescL = m_vecDescL.begin();
    int nDesc = m_vecDescL.size();
    int nKp = m_vecKeyptsL.size();
    int nSizeDesc = nDesc/nKp;

    // estimate matching score
    fMatchingScore = getNearestNeighbour(iterDescL + tpid*nSizeDesc, nSizeDesc, nNei, m_paramIP.m_paramSIFT.m_fDescMatchThr, tpid);

    // collect the pair as a tiept
    if (nNei >= 0) {
        CTiePt tp;
        tp.m_ptIDPair.x = tpid; tp.m_ptIDPair.y = nNei;
        tp.m_fSimVal = fMatchingScore;
        tp.m_ptL = m_vecKeyptsL.at(tpid).pt;
        tp.m_ptR = m_vecKeyptsR.at(nNei).pt;

        // be sure if a matching TP is already in the buffer when using SIFT!!
        // this is because same IP in SIFT can have multiple descriptors
        collectResult(tp);
    }

    return true;
}

static bool compareTps(const CTiePt point1, const CTiePt point2){
    return point1.m_fSimVal < point2.m_fSimVal;
}

float CIPDetector::getMaxMatchingScore(vector<CTiePt>& points){
    auto result = std::max_element(points.begin(), points.end(), compareTps);

    return points.at(std::distance(points.begin(), result)).m_fSimVal;
}

bool CIPDetector::matchDescriptors(bool bIsSURF){
    cout << "Started feature matching" << endl;

    size_t nLength = m_vecKeyptsL.size();
    m_vecTPs.clear();

    std::vector<int> idx ;
    for(int i = 0; i < (int) nLength; i++)
        idx.push_back(i);

    cout << "Starting matcher" << endl;
    // QtConcurrent::blockingMap(idx, [this](const int i){ parallelCompare(i);} );

    //#pragma omp parallel for (sefgault)
    for(int i = 0; i < (int) nLength; i++){
        parallelCompare(i);
    }

    float fMaxDist = getMaxMatchingScore(m_vecTPs);

    // normalising matching scores
    int nTpSz = m_vecTPs.size();
    for (int i = 0; i < nTpSz; i++){
        m_vecTPs.at(i).m_fSimVal /= fMaxDist;
    }

    return true;
}

void CIPDetector::collectResult(CTiePt tp){

    vector<CTiePt>::iterator itr;

   int nLen = m_vecTPs.size();

   if (nLen > 0){
       itr = find(m_vecTPs.begin(), m_vecTPs.end(), tp);
//       CTiePt tp1 = *(itr);
//       CTiePt tp2 = *(itr-1);
       if (itr == m_vecTPs.end() ){
           if (!(m_vecTPs.at(m_vecTPs.size()-1) == tp))
               m_vecTPs.push_back(tp);
       }
       else {
           if((*itr).m_fSimVal > tp.m_fSimVal){
               m_vecTPs.erase(itr);
               m_vecTPs.push_back(tp);
           }
       }
   }
   else
       m_vecTPs.push_back(tp);
}


float CIPDetector::getNearestNeighbour(const vector<float>::iterator iterDescL, const int nSizeDesc,
                                        const int nClassID, int& nNei, float fMatchingThr, int nLIndex){
    int nNeighbour = -1;
    float fDist, fBestDist = 1e6, fSecondBestDist = 1e6;
    vector<float>::iterator iterDescR = m_vecDescR.begin();
    int nLength = m_vecKeyptsR.size();
    double epithresh = 0.01*m_imgL.rows;

    for (int i = 0 ; i < nLength; i++){

        if( nClassID != m_vecKeyptsR.at(i).class_id)
            continue;

        if(fabs(m_vecKeyptsR.at(i).pt.y - m_vecKeyptsL.at(nLIndex).pt.y) >= epithresh)
            continue;

        fDist = compareDescriptors(iterDescL, iterDescR+(i*nSizeDesc), nSizeDesc, fSecondBestDist);

        if( fDist < fBestDist )
        {
            fSecondBestDist = fBestDist;
            fBestDist = fDist;
            nNeighbour = i;
        }
        else if ( fDist < fSecondBestDist )
            fSecondBestDist = fDist;
    }

    if (fBestDist < fMatchingThr*fSecondBestDist)
        nNei = nNeighbour;
    return fBestDist;
}

float CIPDetector::getNearestNeighbour(const vector<float>::iterator iterDescL, const int nSizeDesc, int& nNei, float fMatchingThr, int nLIndex){
    int nNeighbour = -1;
    float fDist, fBestDist = 1e6, fSecondBestDist = 1e6;
    vector<float>::iterator iterDescR = m_vecDescR.begin();
    int nLength = m_vecKeyptsR.size();
    double epithresh = 0.01*m_imgL.rows;

    for (int i = 0 ; i < nLength; i++){

        /* Discard if points are separated by greater than some epipolar threshold */
        if(fabs(m_vecKeyptsR.at(i).pt.y - m_vecKeyptsL.at(nLIndex).pt.y) >= epithresh)
            continue;

        /* SIFT distance */
        fDist = compareDescriptors(iterDescL, iterDescR+(i*nSizeDesc), nSizeDesc, fSecondBestDist);

        if( fDist < fBestDist )
        {
            fSecondBestDist = fBestDist;
            fBestDist = fDist;
            nNeighbour = i;
        }
        else if ( fDist < fSecondBestDist )
            fSecondBestDist = fDist;
    }

    if (fBestDist < fMatchingThr*fSecondBestDist)
        nNei = nNeighbour;
    return fBestDist;
}

float CIPDetector::compareDescriptors(const vector<float>::iterator iterDescL, const vector<float>::iterator iterDescR, const int nSizeDesc, const float fSecondBestDist){

    float fTotalCost = 0.f;

    for (int i = 0 ; i < nSizeDesc; i++){

        float fCost = (*(iterDescL+i) - *(iterDescR+i));
        fCost *= fCost;
        fTotalCost += fCost;

        if (fTotalCost > fSecondBestDist) // no reason to compute more as its result will be discarded anyway!
            break;
    }
    return fTotalCost;
}

Mat CIPDetector::preprocessInputImg(bool bIsLeft){
    // prepare an input image in pgm format (grey)
    Mat imgGrey, imgSrc;
    if (bIsLeft) imgSrc = m_imgL;
    else imgSrc = m_imgR;

    if (imgSrc.channels() > 1)
        cvtColor(imgSrc, imgGrey, cv::COLOR_RGB2GRAY, 1); // make it single chanel
    else
        imgGrey = imgSrc;
    return imgGrey;
}

bool CIPDetector::getSIFTFeatures(bool bIsLeft){ // obtain DoG SIPs and its descriptors
    /////////////////////////////////////////////////////////////////////////
    // prepare an input image in pgm format (grey)
    Mat imgGrey = preprocessInputImg(bIsLeft);
    /*, imgSrc;
    if (bIsLeft) imgSrc = m_imgL;
    else imgSrc = m_imgR;

    if (imgSrc.channels() > 1)
        cvtColor(imgSrc, imgGrey, CV_RGB2GRAY, 1); // make it single chanel
    else
        imgGrey = imgSrc;*/

    vector<int> vecParam;
    vecParam.push_back(cv::IMWRITE_PXM_BINARY);
    vecParam.push_back(1); // set 0 if wanting ascii
    imwrite(m_strResDir + m_paramIP.m_paramProj.m_strSep +"temp.pgm", imgGrey, vecParam);

    // prepare output file name
    string strFrame;
    if (bIsLeft) strFrame = FILE_SIFT_FRAME_L;
    else strFrame = FILE_SIFT_FRAME_R;

    // set processing arguments
    ostringstream strsCmd;
#ifdef _WIN32
    strsCmd << "cmd.exe /c " << m_strExtEXEPath << " < " << m_strResDir << m_paramIP.m_paramProj.m_strSep << "temp.pgm" << " > " << m_strResDir <<  m_paramIP.m_paramProj.m_strSep << strFrame;
#else
    strsCmd << m_strExtEXEPath << " -O " << m_paramIP.m_paramSIFT.m_nOctaves
            << " -S " << m_paramIP.m_paramSIFT.m_nLayers
            << " --first-octave " << m_paramIP.m_paramSIFT.m_nStartingOct
            << " --edge-thresh " << m_paramIP.m_paramSIFT.m_fEdgeThr
            << " --peak-thresh " << m_paramIP.m_paramSIFT.m_fContThr
            << " " << m_strResDir <<  m_paramIP.m_paramProj.m_strSep << "temp.pgm"
            << " -o " << m_strResDir <<  m_paramIP.m_paramProj.m_strSep << strFrame;
#endif

    std::cout << strsCmd.str() << std::endl;

    // execute processing
    system(strsCmd.str().c_str());

    /////////////////////////////////////////////////////////////////////////
    // collect results into memory
    string strFileName = m_strResDir+ m_paramIP.m_paramProj.m_strSep + strFrame;
    ifstream sfFrameL;
    sfFrameL.open(strFileName.c_str());
    if (sfFrameL.is_open()){
        // which image we are processing? l/r?
        vector<float>* pvecDesc;
        vector<KeyPoint>* pvecKP;
        if(bIsLeft){
            pvecDesc = &m_vecDescL;
            pvecKP = &m_vecKeyptsL;
        }
        else {
            pvecDesc = &m_vecDescR;
            pvecKP = &m_vecKeyptsR;
        }
        // collect data

#ifdef _WIN32
        int rows, cols;
        sfFrameL >> rows >> cols;
#endif

        while (!sfFrameL.eof()){
            KeyPoint kp;
#ifdef _WIN32
    sfFrameL >> kp.pt.y >> kp.pt.x >>kp.size >> kp.angle;
    // For some reasone Lowe's binary swaps the order of x/y depending on platform....
#else
    sfFrameL >> kp.pt.x >> kp.pt.y >>kp.size >> kp.angle; // get key poiints
#endif

            pvecKP->push_back(kp);

           // descriptor
            for (int i = 0 ; i < 128; i++){
                float fBinVal;
                sfFrameL >> fBinVal;
                pvecDesc->push_back(fBinVal);
            }
        }

        pvecKP->pop_back(); // Remove the last one because we don't need it.

        for (int i = 0 ; i < 128; i++)
            pvecDesc->pop_back();

        sfFrameL.close();

    }
    else{
        return false;
    }

    return true;
}
