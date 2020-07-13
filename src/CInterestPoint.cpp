#include "CInterestPoint.h"
#include <sys/stat.h>

CInterestPoint::CInterestPoint()
{

}

CInterestPoint::CInterestPoint(Mat matImg, CIPParamAdapt* pparam){
    setInputImage(matImg);
    setProcParam(pparam);
}

CInterestPoint::CInterestPoint(Mat matImg, CIPParamAdapt* pparam, string strRscPath){
    setInputImage(matImg);
    setProcParam(pparam);
    setResourceDir(strRscPath);
}

//CInterestPoint::CInterestPoint(Mat matImg, string strAllRes){
//    m_matImgSrc = matImg; // header copy not hard copy
//    m_vecIPs.empty();
//    m_vecDesc.empty();
//    ifstream sfRes;
//    string strFileName = strAllRes;
//    sfRes.open(strFileName.c_str());
//    if (sfRes.is_open()){
//        int nSzDesc = this->m_vecDesc.size()/m_vecIPs.size();
//        for (int i = 0; i < (int) m_vecIPs.size(); i ++){
//            KeyPoint kp = m_vecIPs.at(i);
//            sfDesc << kp.pt.x << " " << kp.pt.y << " "
//                   << kp.size << " " << kp.angle << " " << kp.class_id << " ";
//            for (int j = 0; j < nSzDesc; j++){
//                sfDesc << m_vecDesc.at(i*nSzDesc+j) << " ";
//            }
//            sfDesc << endl;
//        }
//        sfRes.close();
//     }
//    else
//        return false;
//    return true;
//}

//void CInterestPoint::setResourceDir(string strRscPath) {
//    string strFile = strRscPath+"sift";
//    if(isFileExists(strFile)){
//        m_strRscPath = strRscPath;
//    }
//    else{
//        cout << "Warning: SIFT file is not found in the provided resource directory!" << endl;
//    }
//}

bool CInterestPoint::isFileExists(string strFile){
    struct stat stFileInfo;
    bool blnReturn;
    int intStat;

    // Attempt to get the file attributes
    intStat = stat(strFile.c_str(),&stFileInfo);
    if(intStat == 0) {
        // We were able to get the file attributes
        // so the file obviously exists.
        blnReturn = true;
    } else {
        // We were not able to get the file attributes.
        // This may mean that we don't have permission to
        // access the folder which contains this file. If you
        // need to do that level of checking, lookup the
        // return values of stat which will give you
        // more details on why stat failed.
        blnReturn = false;
    }

    return(blnReturn);
}

bool CInterestPoint::detectIPs(){
    if (!m_pparamIP || !m_matImgSrc.data){
        cout << "Error: Please check if the input image or processing parameters are correct" << endl;
        return false;
    }

    bool bRes = false;

    if (m_pparamIP->m_nType == CIPParamAdapt::IP_DOG)
        bRes = getSIFTFeatures();
    else if (m_pparamIP->m_nType == CIPParamAdapt::IP_DOH)
        cout << "Surf not implemented yet";
    else
        cout << "Warning: Unknown IP type" << endl;

    return bRes;
}
Mat CInterestPoint::preprocessInputImg(){
    // prepare a grey input image
    Mat imgGrey;
    if (m_matImgSrc.channels() > 1)
        cvtColor(m_matImgSrc, imgGrey, cv::COLOR_RGB2GRAY, 1); // make it single chanel grey image
    else
        imgGrey = m_matImgSrc;
    return imgGrey;
}

bool CInterestPoint::getSIFTFeatures(){

    clearResultBuffers();

    // prepare output file name
    const string strFrame = "SIFTFrame.txt";            // IP + Descriptor
    const string strImgFile = "tempImg.pgm";            // temp image
    const string strExtEXEPath = m_strRscPath + "sift"; // the location of an external SIFT executable
    const string strTempDir = m_strRscPath;

    if (!isFileExists(strExtEXEPath)) {
        cout << "Error: SIFT executable is not found in the resource directory, " << endl;
        cout << strExtEXEPath << endl;
        return false;
    }

    /////////////////////////////////////////////////////////////////////////
    // prepare an input image in pgm format (grey)
    /////////////////////////////////////////////////////////////////////////
    Mat imgGrey = preprocessInputImg();
    vector<int> vecParam;  // processing parameters used for imwrite
    vecParam.push_back(cv::IMWRITE_PXM_BINARY);
    vecParam.push_back(1); // set 0, if ASCII format is required
    imwrite(strTempDir + strImgFile, imgGrey, vecParam);

    /////////////////////////////////////////////////////////////////////////
    // set processing arguments
    /////////////////////////////////////////////////////////////////////////
    ostringstream strsCmd;
    strsCmd << strExtEXEPath
            << " -O " << m_pparamIP->m_nOctaves
            << " -S " << m_pparamIP->m_nLayers
            << " --first-octave " << m_pparamIP->m_nStartingOct
            << " --edge-thresh " << m_pparamIP->m_fEdgeThr
            << " --peak-thresh " << m_pparamIP->m_fContThr
            << " " << strTempDir << strImgFile
            << " -o " << strTempDir << strFrame;

    // execute processing
    system(strsCmd.str().c_str());

    /////////////////////////////////////////////////////////////////////////
    // collect the results in the file system to the system memory
    /////////////////////////////////////////////////////////////////////////
    string strFileName = strTempDir + strFrame;
    ifstream sfFrame;
    sfFrame.open(strFileName.c_str());
    if (sfFrame.is_open()){
        vector<float>* pvecDesc;
        vector<KeyPoint>* pvecKP;
        pvecDesc = &m_vecDesc;
        pvecKP = &m_vecIPs;

        // collect data
        while (!sfFrame.eof()){
            KeyPoint kp;
            sfFrame >> kp.pt.x >> kp.pt.y >>kp.size >> kp.angle; // get key poiints
            pvecKP->push_back(kp);

           // get descriptor
            for (int i = 0 ; i < 128; i++){
                float fBinVal;
                sfFrame >> fBinVal;
                pvecDesc->push_back(fBinVal);
            }
        }

        pvecKP->pop_back();
        for (int i = 0 ; i < 128; i++)
            pvecDesc->pop_back();

        sfFrame.close();

        // delete SIFT result in "strTempDir + strFrame" and "strTempDir + strImgFile"

    }
    else
        return false;

    return true;
}

bool CInterestPoint::saveIPRes(string strPath){

    // prepare matrix;
//    int nKpLen = m_vecIPs.size();
//    Mat matKPs = Mat::zeros(nKpLen, 7, CV_32FC1);
//    for (int i = 0 ; i < nKpLen; i++){
//        KeyPoint kp = m_vecIPs.at(i);
//        matKPs.at<float>(i,0) = kp.pt.x;
//        matKPs.at<float>(i,1) = kp.pt.y;
//        matKPs.at<float>(i,2) = kp.size;
//        matKPs.at<float>(i,3) = kp.angle;
//        matKPs.at<float>(i,4) = kp.class_id;
//        matKPs.at<float>(i,5) = kp.octave;
//        matKPs.at<float>(i,6) = kp.response;
//    }
//    FileStorage fs(strPath, FileStorage::WRITE);
//    if(fs.isOpened()){
//        fs << "KeyPoints"<<matKPs;
//        fs.release();
//    }

    ofstream sfIP;
    string strFileName = strPath;//+"IP.txt";
    sfIP.open(strFileName.c_str());

    if (sfIP.is_open()){
        int nIPLen = (int) m_vecIPs.size();
        sfIP << nIPLen << " " << 5 << endl; // header
        for (int i = 0; i < nIPLen; i++){
            KeyPoint kp = m_vecIPs.at(i);
            sfIP << kp.pt.x << " " << kp.pt.y << " "
                 << kp.size << " " << kp.angle << " "
                 << kp.class_id << endl;
        }
        sfIP.close();
     }
    else
        return false;

    return true;
}

bool CInterestPoint::saveDescRes(string strPath){

    ofstream sfDesc;
    string strFileName = strPath;//+"desc.txt";
    sfDesc.open(strFileName.c_str());

    if (sfDesc.is_open()){

        int nIPLen = m_vecIPs.size();
        int nSzDesc = m_vecDesc.size()/nIPLen;

        sfDesc << nIPLen << " " << nSzDesc << endl; // header

        for (int i = 0; i < nIPLen; i ++){
            for (int j = 0; j < nSzDesc; j++){
                sfDesc << m_vecDesc.at(i*nSzDesc+j) << " ";
            }
            sfDesc << endl;
        }
        sfDesc.close();
     }
    else
        return false;

    return true;
}

bool CInterestPoint::saveAllResults(string strPath){
    ofstream sfRes;
    string strFileName = strPath;//+"desc.txt";
    sfRes.open(strFileName.c_str());

    if (sfRes.is_open()){
        int nIPLen = (int) m_vecIPs.size();
        int nSzDesc = m_vecDesc.size()/nIPLen;

        sfRes << nIPLen << " " << nSzDesc+5 << endl; // header

        for (int i = 0; i < nIPLen; i ++){
            KeyPoint kp = m_vecIPs.at(i);
            sfRes << kp.pt.x << " " << kp.pt.y << " "
                  << kp.size << " " << kp.angle << " " << kp.class_id << " ";
            for (int j = 0; j < nSzDesc; j++){
                sfRes << m_vecDesc.at(i*nSzDesc+j) << " ";
            }
            sfRes << endl;
        }
        sfRes.close();
     }
    else
        return false;

    return true;
}

bool CInterestPoint::setResult(string strAllRes){
    m_vecIPs.empty();
    m_vecDesc.empty();

    ifstream sfRes;
    sfRes.open(strAllRes.c_str());
    if (sfRes.is_open()){
        int nLine = 0;
        int nCol = 0;
        sfRes >> nLine >> nCol;

        int nCount = 0;
        while(nCount < nLine){
            KeyPoint kp;
            //float pDesc[nCol];

            sfRes >> kp.pt.x >> kp.pt.y >> kp.size >> kp.angle >> kp.class_id;
            m_vecIPs.push_back(kp);

            for (int i = 0; i < nCol-5; i++){
                //sfRes >> pDesc[i];
                float fDesc = 0;
                sfRes >> fDesc;
                m_vecDesc.push_back(fDesc);
            }
            nCount++;
        }

        // create Processing param
//        m_pparamIP = new CIPParam;
//        if (nCol > 64)
//            m_pparamIP->setSIFTParam();
//        else
//            m_pparamIP->setSURFParam();
//        bDelete = true;
    }
    else
        return false;

    return true;
}


vector<KeyPoint> CInterestPoint::getClonedIPs() const {
    vector<KeyPoint> vecRes;

    for (int i = 0; i < (int) m_vecIPs.size(); i++){
        KeyPoint kp = m_vecIPs.at(i);
        vecRes.push_back(kp);
    }

    return vecRes;
 }

vector<float> CInterestPoint::getClonedDescriptors() const{
    vector<float> vecRes;

    for (int i = 0; i < (int) m_vecDesc.size(); i++){
        vecRes.push_back(m_vecDesc.at(i));
    }

    return vecRes;
}

vector<CTiePt> CInterestPoint::getTCfromDescMatching(CInterestPoint& ipRight){

    vector<CTiePt> vecRes;

    if ( getTypeOfIP() != ipRight.getTypeOfIP() ){
        cout << "Error: Two descriptors should be indential type" << endl;
    }
    else if (getTypeOfIP() == CIPParamAdapt::IP_UNKNOWN){
        cout << "Error: Unknown descriptor type" << endl;
    }
    else{
        int nKp = m_vecIPs.size();
        const vector<KeyPoint>* pvecIPs = ipRight.getIPs();
        const vector<float>* pvecDesc = ipRight.getDescriptor();

        for (int i = 0 ; i < nKp; i++){
            int nNei = -1;
            float fMatchingScore = -1.f;

            // estimate matching score
            if (getTypeOfIP() == CIPParamAdapt::IP_DOG) // for SIFT
                fMatchingScore = getMatchingDesc(i, pvecDesc, nNei);
            else // for SURF
                fMatchingScore = getMatchingDesc(i, pvecDesc, pvecIPs, m_vecIPs.at(i).class_id, nNei);

            // collect the pair as a tiept
            if (nNei >= 0) {
                CTiePt tp;
                tp.m_fSimVal = fMatchingScore;
                tp.m_ptL = m_vecIPs.at(i).pt;
                tp.m_ptR = pvecIPs->at(nNei).pt;
                tp.m_ptIDPair = Point2i(i,nNei);

                // be sure if a matching TP is already in the buffer when using SIFT!!
                // this is because same IP in SIFT can have multiple descriptors
                if (getTypeOfIP() == CIPParamAdapt::IP_DOH)
                    vecRes.push_back(tp);
                else{        // When SIFT decriptor matching
                    vector<CTiePt>::iterator itr;
                    int nLen = vecRes.size();
                    if (nLen > 0){
                        itr = find(vecRes.begin(), vecRes.end(), tp);

                        if (itr == vecRes.end() ){
                            if (!((*itr) == tp))
                                vecRes.push_back(tp);
                          }
                        else {
                            if((*itr).m_fSimVal > tp.m_fSimVal){
                                vecRes.erase(itr);
                                vecRes.push_back(tp);
                            }
                        }
                    }
                    else
                        vecRes.push_back(tp);
                }
            }
        }
    }

    return vecRes;
}

float CInterestPoint::getMatchingDesc(const int nPos, const vector<float>* pvecDescR, int& nNei){
    int nNeighbour = -1;
    float fDist, fBestDist = 1e6, fSecondBestDist = 1e6;
    //vector<float>::iterator iterDescR = pvecDescR->begin();
    int nDesc = m_vecDesc.size() / m_vecIPs.size();
    int nLength = pvecDescR->size()/nDesc;

    for (int i = 0 ; i < nLength; i++){

        //fDist = compareDescriptors(iterDescL, iterDescR+(i*nSizeDesc), nSizeDesc, fSecondBestDist);
        fDist = compareDescriptors( nPos, i, pvecDescR, fSecondBestDist);

        if( fDist < fBestDist )
        {
            fSecondBestDist = fBestDist;
            fBestDist = fDist;
            nNeighbour = i;
        }
        else if ( fDist < fSecondBestDist )
            fSecondBestDist = fDist;

    }

    if (fBestDist < 0.6*fSecondBestDist)
        nNei = nNeighbour;

    return fBestDist;
}

float CInterestPoint::compareDescriptors(const int nPosL, const int nPosR,
                                         const vector<float>* pvecDescR, const float fSecondBestDist){

    float fTotalCost = 0.f;
    int nSizeDesc =  m_vecDesc.size() / m_vecIPs.size();
    vector<float>::iterator iterDescL = m_vecDesc.begin() + nPosL*nSizeDesc;
    //vector<float>::iterator iterDescR = pvecDescR->begin() + nPosR*nSizeDesc; // causing error why?

    for (int i = 0 ; i < nSizeDesc; i++){
        float fCost = (*(iterDescL+i) - pvecDescR->at(nPosR*nSizeDesc+i)); //(*(iterDescL+i) - *(iterDescR+i));
        fCost *= fCost;
        fTotalCost += fCost;

        if (fTotalCost > fSecondBestDist) // No reason to compute more as its result is already higeher than the secondBest,
                                          // so that it will be discarded anyway!
            break;
    }

    return fTotalCost;
}

float CInterestPoint::getMatchingDesc(const int nPos, const vector<float>* pvecDescR, const vector<KeyPoint>* pvecIPR,
                                      const int nClassID, int& nNei){

    int nNeighbour = -1;
    float fDist, fBestDist = 1e6, fSecondBestDist = 1e6;
    //vector<float>::iterator iterDescR = m_vecDescR.begin();
//    int nLength = m_vecIPs.size();
    int nDesc = pvecDescR->size() / pvecIPR->size();
    int nLength = pvecDescR->size()/nDesc;

    for (int i = 0 ; i < nLength; i++){

        if(nClassID != pvecIPR->at(i).class_id)
           continue;

        fDist = compareDescriptors(nPos, i, pvecDescR, fSecondBestDist);

        if(fDist < fBestDist)
        {
            fSecondBestDist = fBestDist;
            fBestDist = fDist;
            nNeighbour = i;
        }
        else if ( fDist < fSecondBestDist )
            fSecondBestDist = fDist;
    }

    if (fBestDist < 0.6*fSecondBestDist)
        nNei = nNeighbour;
    return fBestDist;

}

bool CInterestPoint::saveTPResult(const vector<CTiePt>& vecTP, string strFile, bool bDistortionParam) const{

    ofstream sfTC;
    sfTC.open(strFile.c_str());

    if (sfTC.is_open()){

        int nSzTC = vecTP.size();

        sfTC << nSzTC << " "<<13 << endl;

        for (int i = 0; i < nSzTC; i ++){
            CTiePt tp = vecTP.at(i);
            sfTC << tp.m_ptIDPair.x << " " << tp.m_ptIDPair.y << " "
                 << tp.m_ptL.x << " " <<  tp.m_ptL.y << " "
                 << tp.m_ptR.x << " " <<  tp.m_ptR.y << " "
                 << tp.m_fSimVal; // << endl;

            if (bDistortionParam){
                for (int i = 0; i < 4; i++)
                    sfTC << " " << tp.m_pfAffine[i];

                sfTC << " " << tp.m_ptOffset.x << " " <<  tp.m_ptOffset.y;
            }
            sfTC << endl;
        }
        sfTC.close();
     }
    else
        return false;

    return true;
}
