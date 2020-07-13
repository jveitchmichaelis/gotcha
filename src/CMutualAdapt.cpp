#include "CMutualAdapt.h"
#include "CAlscAdapt.h"

CMutualAdapt::~CMutualAdapt()
{
    if(m_bDeletePointer){
        delete m_pIpL; // ip and descriptor
        delete m_pIpR;
        delete [] m_pdAutoShapeParamL; // this is transfrom that change a circle to an ellipse
        delete [] m_pdAutoShapeParamR;
    }
}

CMutualAdapt::CMutualAdapt(string strImgL, string strImgR, string strIPL, string strIPR, string strAutoShapeL, string strAutoShapeR){
    // this constructor is often used for debugging
    // load images
    m_matImgL = imread(strImgL, 0);
    m_matImgR = imread(strImgR, 0);

    m_bDeletePointer = true;
    m_fDescMatchingThr = 0.6f;

    // load IPs
    m_pIpL = new CInterestPoint;
    m_pIpL->setInputImage(m_matImgL);
    m_pIpL->setResult(strIPL);

    m_pIpR = new CInterestPoint;
    m_pIpR->setInputImage(m_matImgR);
    m_pIpR->setResult(strIPR);

    // load auto shape
    loadAutoShape(strAutoShapeL);
    loadAutoShape(strAutoShapeR, false);
}

bool CMutualAdapt::loadAutoShape(string strFile, bool bIsLeft){
    ifstream sfAff;
    sfAff.open(strFile.c_str());

    if (sfAff.is_open()){
        int nRow = 0, nCol = 0;
        sfAff >> nRow >> nCol;
        double* dpData = NULL;

        if (bIsLeft){
            m_pdAutoShapeParamL = new double[nRow*4];
            dpData = m_pdAutoShapeParamL;
        }else{
            m_pdAutoShapeParamR = new double[nRow*4];
            dpData = m_pdAutoShapeParamR;
        }

        // collect data
        for (int i = 0; i< nRow; i++){
            float x,y,scale;
            float a1,a2,a3,a4;
            sfAff >> x >> y >> scale; // dummy values

            sfAff >> a1 >> a2 >> a3 >> a4;
            dpData[i*4] = a1;
            dpData[i*4+1] = a2;
            dpData[i*4+2] = a3;
            dpData[i*4+3] = a4;
// Debug
//            if (bIsLeft){
//            for (int k = 0 ; k < 4; k++)
//                cout << m_pdAutoShapeParamL[i*4+k] << " ";
//            }
//            else {
//                for (int k = 0 ; k < 4; k++)
//                    cout << m_pdAutoShapeParamR[i*4+k] << " ";
//            }
//            cout << endl;
// Debug end
        }
    }
    else
        return false;

    return true;
}

void CMutualAdapt::doMutualAdaptationForRefinement(){
    if (m_pIpL->getDescriptorLength() != m_pIpR->getDescriptorLength()){
        // Mutual adaptation can be used even if the type of the IP descriptors are different,
        // but it would be a very slow process as it'll try to match every possible pair
        cout << "two input IPs are from the same method!" << endl;
        return;
    }

    m_vecTP.empty(); // clear the result buffer
    int nIPLen = m_pIpL->getNumOfIPs();//IPs()->size();
    for (int i = 0; i < nIPLen; i++){
        CTiePtAdapt tp;
        if ( !validateMatch(i, i, tp) )
            continue; // if the candidate is not verified by ALSC then just discard it
        else {
            collectTP(tp);
        }
    }
}

void CMutualAdapt::doMutualAdaptation(){

    if (m_pIpL->getDescriptorLength() != m_pIpR->getDescriptorLength()){
        // Mutual adaptation can be used even if the type of the IP descriptors are different,
        // but it would be a very slow process as it'll try to match every possible pair
        cout << "two input IPs are from the same method!" << endl;
        return;
    }

    m_vecTP.empty(); // clear the result buffer

    int nIPLen = m_pIpL->getNumOfIPs();//IPs()->size();
    for (int i = 0; i < nIPLen; i++){
        // show the progress
//        cout << i/(float(nIPLen))*100 <<"%" << endl;

// debug
//    for (int i = 73; i < 74; i++){
//        for (int i = 1398; i < 1399; i++){
//            for (int i = 1550; i < 1551; i++){
// debug  end

        // collect matching candidates from the right
        // based on the descriptor matching
        int nPosR = getMatchingCandidates(i);
        if (nPosR < 1) continue;

// debug
//        cout <<i<<": " << nPosR << endl;
// debug end

        CTiePtAdapt tp;
        if ( !validateMatch(i, nPosR, tp) )
            continue; // if the candidate is not verified by ALSC then just discard it
        else {
////            getRefined(tp);
////            // collect the best candidate for TP
////            // check if it is already collected (happens with SIFT as it has multiple descriptor at the same position)
            collectTP(tp);
        }
    }

    // clean up to make one-one matching before ransac
//    makeOneToOne();
}

int CMutualAdapt::getMatchingCandidates(const int nPosL){

    int nRes = -1;

    float fDist, fBestDist = 1e6, fSecondBestDist = 1e6;
    float fWeight = m_fDescMatchingThr;//0.6f;

    const vector<float>* pvecDescR = m_pIpR->getDescriptor();
    int nIPr = m_pIpR->getNumOfIPs();

    int nBestIP = -1;
    for (int i = 0; i < nIPr; i++){
        fDist = compareDescriptors(nPosL, i, pvecDescR, fSecondBestDist);

        if(fDist < fBestDist){
            fSecondBestDist = fBestDist;
            fBestDist = fDist;
            nBestIP = i;
        }
        else if (fDist < fSecondBestDist)
            fSecondBestDist = fDist;
    }

    if (fBestDist < fWeight*fSecondBestDist)
        nRes = nBestIP;

    return nRes;
}

float CMutualAdapt::compareDescriptors(const int nPosL, const int nPosR,
                                       const vector<float>* pvecDescR, const float fSecondBestDist){

    float fTotalCost = 0.f;
    int nSizeDesc = m_pIpL->getDescriptorLength(); // believe that the length of left and right descriptor are identical
    const vector<float>* pvecDescL = m_pIpL->getDescriptor();

    if (m_pIpR->getDescriptorLength() != 128){ // if descriptor is from SURF there is another condition to check for the speedup matching
        KeyPoint kpL = m_pIpL->getIPs()->at(nPosL);
        KeyPoint kpR = m_pIpR->getIPs()->at(nPosR);
        if (kpL.class_id != kpR.class_id)
            return fSecondBestDist*100; // don't need to compute the distace anymore
    }

    for (int i = 0 ; i < nSizeDesc; i++){
        float fCost = (pvecDescL->at(nPosL*nSizeDesc+i) - pvecDescR->at(nPosR*nSizeDesc+i));

        fCost *= fCost;
        fTotalCost += fCost;

        if (fTotalCost > fSecondBestDist) // No reason to compute more as its result is already higeher than the secondBest,
            break;                        // so that it will be discarded anyway!
    }

    return fTotalCost;
}

void CMutualAdapt::saveMat(Mat& matIn, string strFile){
    ofstream sfTC;
    sfTC.open(strFile.c_str());

    if (sfTC.is_open()){
        for (int i = 0; i < matIn.rows; i++){
            for (int j = 0 ; j < matIn.cols; j++){
                if (matIn.depth() == CV_64F)
                    sfTC << matIn.at<double>(i,j) << " ";
                else if (matIn.depth() == CV_32F)
                    sfTC << matIn.at<float>(i,j) << " ";
            }
            sfTC << endl;
        }
        sfTC.close();
    }
}

void CMutualAdapt::makeOneToOne(){
    int nLen = m_vecTP.size();
    if (nLen < 1) return;

    vector<CTiePtAdapt> vecTP1;
    for(int i = 0 ; i < nLen; i++){
        CTiePtAdapt tp = m_vecTP.at(i);
        int nPosR = tp.m_ptIDPair.y;
        vector<int> vecID;
        for (int j = 0 ; j < nLen; j++){
            if (nPosR == m_vecTP.at(j).m_ptIDPair.y)
                vecID.push_back(j);

        }
        if (vecID.size() == 1){
            vecTP1.push_back(tp);
        }
    }

    m_vecTP.empty();
    m_vecTP = vecTP1;
}

void CMutualAdapt::collectTP(CTiePtAdapt& tpIn){
    vector<CTiePtAdapt>::iterator itr;
    bool bNeedToAdd = false;
    int nLen = m_vecTP.size();

    if (nLen > 0){
        itr = find(m_vecTP.begin(), m_vecTP.end(), tpIn);

        if (itr == m_vecTP.end() ){
            if (!(m_vecTP.at(m_vecTP.size()-1) == tpIn))
                bNeedToAdd = true; //m_vecTP.push_back(tp);
        }
        else {
            if((*itr).m_fSimVal > tpIn.m_fSimVal){
                m_vecTP.erase(itr); // override the previous record
                //m_vecTP.push_back(tp);
                bNeedToAdd = true;
            }
        }
    }
    else
        bNeedToAdd = true;

    if (bNeedToAdd)
        m_vecTP.push_back(tpIn);
}

bool CMutualAdapt::validateMatch(const int nPosL, const int nPosR, CTiePtAdapt& tp){

    bool bRes = false;

    ///////////////////////////////////////////////////////
    // Estimate ALSC distances for all candidates
    ///////////////////////////////////////////////////////
//    float fScale = 1.f;
    float fSigma = 1.f;
    float fScaleMatchingWindow = 3.f; // this value "3" is used for the SIFT descriptor estimation as well
    float fMaxALSCEig = 1000.0f;

    KeyPoint kpL = m_pIpL->getIPs()->at(nPosL);
    KeyPoint kpR = m_pIpR->getIPs()->at(nPosR);

    //////////////////////////////////////////////////////////
    // 1. geometric normalisation (scale, rot, and affine)
    //////////////////////////////////////////////////////////
    if(kpL.size > kpR.size){
//        fScale = kpR.size/kpL.size;
        fSigma = kpR.size * fScaleMatchingWindow;
    }
    else{
//        fScale = kpL.size/kpR.size;
        fSigma = kpL.size * fScaleMatchingWindow;
    }

    // define the radius of the ALSC matching window
    int nALSCPatchRad = (int) round(fSigma);
    nALSCPatchRad = max(3, nALSCPatchRad);  // minimum ALSC matching window = 3*2+1
    nALSCPatchRad = min(nALSCPatchRad, 30); // maximum ALSC matching window = 30*2+1

    double pdInvNorm[4] = {0,}; // this is a buffer to store the normalising transform parameters (ellipse to circle)
//    double pdInvNormR[4] = {0,};
    Mat matNormImgL, matNormImgR;
    if (kpL.size > kpR.size){
        matNormImgL = getNormalisedPatch(nPosL, nPosR, nALSCPatchRad, pdInvNorm, true);
        matNormImgR = cropImage(nPosR, nALSCPatchRad, false);
    }
    else {
        matNormImgL = cropImage(nPosL, nALSCPatchRad, true);
        matNormImgR = getNormalisedPatch(nPosL, nPosR, nALSCPatchRad, pdInvNorm, false);
    }

//    saveMat(matNormImgL, "/Users/DShin/Desktop/NormImgL.txt");
//    saveMat(matNormImgR, "/Users/DShin/Desktop/NormImgR.txt");

    //////////////////////////////////////////////////////////
    // 2. do ALSC matching
    //////////////////////////////////////////////////////////
    CALSCParam paramALSC;
    paramALSC.m_fEigThr = fMaxALSCEig;
    paramALSC.m_fDriftThr = 1.0f;
    paramALSC.m_bWeighting = false; // make it false or it will be slow
    paramALSC.m_nPatch = nALSCPatchRad*2+1; // ALSC processing parameters

    CALSCAdapt alsc(matNormImgL, matNormImgR, paramALSC);
    Point2f ptL_ALSC(matNormImgL.cols/2, matNormImgL.rows/2);
    Point2f ptR_ALSC(matNormImgR.cols/2, matNormImgR.rows/2);
    alsc.performALSC(ptL_ALSC, ptR_ALSC);
    CTiePt tpALSCOut = alsc.getRefinedTp(0); // be careful: tpALSCOut store result in the normalised plane

    CALSCAdapt alsc2(matNormImgR, matNormImgL, paramALSC);
    alsc2.performALSC(tpALSCOut.m_ptR, tpALSCOut.m_ptL);
    CTiePt tpALSCOut2 = alsc2.getRefinedTp(0); // be careful: tpALSCOut store result in the normalised plane

    //////////////////////////////////////////////////////////
    // 3. convert the ALSC refined co-ordinate back
    //////////////////////////////////////////////////////////
//    CTiePtAdapt tpAdapt;
    if (tpALSCOut.m_fSimVal != CTiePt::NOT_DEF &&
        tpALSCOut2.m_fSimVal != CTiePt::NOT_DEF){

        bRes = true;
        tp.m_ptL = kpL.pt;                  // the left point in the original coordinate
        tp.m_ptR = kpR.pt;
        tp.m_ptIDPair = Point2i(nPosL, nPosR);
        tp.m_fSimVal = (tpALSCOut.m_fSimVal + tpALSCOut2.m_fSimVal) / fMaxALSCEig/2;
        for (int k = 0 ; k < 4; k++){       // The shape distortion from the auto shape adaptation
            tp.m_pfAffine[k] = m_pdAutoShapeParamR[nPosR * 4 + k];
            tp.m_pfAffineL[k] = m_pdAutoShapeParamL[nPosL * 4 + k];
        }

        // collect alsc estimation
        Mat matNormInv(2,2,CV_64FC1,pdInvNorm);

        if (kpL.size > kpR.size){
            tp.m_ptOffset = tpALSCOut.m_ptOffset;          // right offset

            double pdPtOffsetL[2] = {tpALSCOut2.m_ptOffset.x, tpALSCOut2.m_ptOffset.y};
            Mat matPtOffset(2,1,CV_64FC1,pdPtOffsetL);     // NB. this is the offset in the normalised plane
            matPtOffset = matNormInv * matPtOffset;        // convert the ALSC offset in the normalised plane back to the original plane
            tp.m_ptOffsetL.x = matPtOffset.at<double>(0,0);
            tp.m_ptOffsetL.y = matPtOffset.at<double>(1,0);
        }
        else{
            tp.m_ptOffsetL = tpALSCOut2.m_ptOffset;

            double pdPtOffsetR[2] = {tpALSCOut.m_ptOffset.x, tpALSCOut.m_ptOffset.y};
            Mat matPtOffset(2,1,CV_64FC1,pdPtOffsetR);     // NB. this is the offset in the normalised plane
            matPtOffset = matNormInv * matPtOffset;        // convert the ALSC offset in the normalised plane back to the original plane
            tp.m_ptOffset.x = matPtOffset.at<double>(0,0);
            tp.m_ptOffset.y = matPtOffset.at<double>(1,0);
        }

        if (norm(tp.m_ptOffset) > 2 || norm(tp.m_ptOffsetL) > 2) {
            return false;
        }

        /* the following data is not used in the current code */
        // An affine transform from the initial boundary to the ALSC-updated boundary in a normalised R plane
        // collect the transform results
//        Mat matNormR3by3 = Mat::zeros(3,3,CV_64FC1); // right normalisation TR
//        matNormR3by3.at<double>(0,0) = pdAffR[0];
//        matNormR3by3.at<double>(0,1) = pdAffR[1];
//        matNormR3by3.at<double>(1,0) = pdAffR[2];
//        matNormR3by3.at<double>(1,1) = pdAffR[3];
//        matNormR3by3.at<double>(2,2) = 1.f;
//        tp.m_matNormR = matNormR3by3;
//        Mat matNormL3by3 = Mat::zeros(3,3,CV_64FC1); // left normalisation TR
//        matNormL3by3.at<double>(0,0) = pdAffL[0];
//        matNormL3by3.at<double>(0,1) = pdAffL[1];
//        matNormL3by3.at<double>(1,0) = pdAffL[2];
//        matNormL3by3.at<double>(1,1) = pdAffL[3];
//        matNormL3by3.at<double>(2,2) = 1.f;
//        tp.m_matNormL = matNormL3by3;

        // A * m_matALSCAffL*NormL*X_l = m_matALSCAffR*NormR*X_r
        tp.m_matALSCAff = alsc.getAffineTR(0, nALSCPatchRad);
        tp.m_matALSCAffL = alsc2.getAffineTR(0, nALSCPatchRad);
//        tp.m_matAff_LR = getAffineTR(tp, nALSCPatchRad); // affine tr between normalised L and normalised R
    }

    return bRes;
}

Mat CMutualAdapt::cropImage(const int nPos, const int nALSCPatchRad, bool isLeft){

    Mat matImgSrc;
    KeyPoint kp;
    if (isLeft){
        kp = m_pIpL->getIPs()->at(nPos);
        matImgSrc = m_matImgL;
    }
    else{
        kp = m_pIpR->getIPs()->at(nPos);
        matImgSrc = m_matImgR;
    }
    float fScaleCropImg = 3.f;
    int nRadPatch = (int) round(nALSCPatchRad * fScaleCropImg);
    Mat matImgCrop = Mat::zeros(2*nRadPatch+1, 2*nRadPatch+1, CV_32FC1);
    matImgCrop.at<float>(nRadPatch, nRadPatch) = getSubPixelValue(kp.pt, matImgSrc);

    for (int i = -nRadPatch ; i <= nRadPatch; i++){
            for (int j = -nRadPatch ; j <= nRadPatch; j++){
                if (i == 0 && j == 0 ) continue; // the centre of the patch
                int y = i+nRadPatch; // x position in the patch
                int x = j+nRadPatch;
                Point2f ptDest(j+kp.pt.x, i+kp.pt.y);
//                double pdData[2] = {j, i};
//                Mat matTemp(2,1, CV_64FC1, pdData);
//                Mat matNewPos = matInvTr * (matTRPos + matTemp);
//                matImgCrop.at<float>(y,x) = getSubPixelValue(Point2f(matNewPos.at<double>(0,0),matNewPos.at<double>(1,0)), matImgSrc);
                matImgCrop.at<float>(y,x) = getSubPixelValue(ptDest, matImgSrc);
            }
        }

    return matImgCrop;
}

Mat CMutualAdapt::getNormalisedPatch(const int nPosL, const int nPosR, const int nALSCPatchRad, double* pdInvNorm, bool bIsLeft){

    ///////////////////////////////////
    // make an normalisation transform
    ///////////////////////////////////
    Mat matImgSrc;
    KeyPoint kpL = m_pIpL->getIPs()->at(nPosL);
    KeyPoint kpR = m_pIpR->getIPs()->at(nPosR);
    float fScale = 1.f;
    double pdAutoAffL[4] ={1,0,0,1};
    double pdAutoAffR[4] ={1,0,0,1};
    double dAngleL = 0;
    double dAngleR = 0;
    Point2f ptCen;

    //////////////////////////////////////////////////////////
    // 1. geometric normalisation (scale, rot, and affine)
    //////////////////////////////////////////////////////////
    if(bIsLeft){  // ifkpL.size > kpR.size)
        matImgSrc = m_matImgL;
        fScale = kpR.size/kpL.size;
        ptCen = kpL.pt;
    }
    else {
        matImgSrc = m_matImgR;
        fScale = kpL.size/kpR.size;
        ptCen = kpR.pt;
    }

    for (int i = 0; i < 4; i++){
        pdAutoAffL[i] = m_pdAutoShapeParamL[nPosL*4+i];
        pdAutoAffR[i] = m_pdAutoShapeParamR[nPosR*4+i];
    }
    dAngleL = kpL.angle;
    dAngleR = kpR.angle;
    double pdRotL[4] = {cos(dAngleL), sin(dAngleL), -sin(dAngleL), cos(dAngleL)};
    double pdRotR[4] = {cos(dAngleR), sin(dAngleR), -sin(dAngleR), cos(dAngleR)};

    // prepare a transform
    Mat matRotL(2,2,CV_64FC1, pdRotL);
    Mat matRotR(2,2,CV_64FC1, pdRotR);
    Mat matAutoAffL(2,2,CV_64FC1, pdAutoAffL);
    Mat matAutoAffR(2,2,CV_64FC1, pdAutoAffR);

    Mat matTr = Mat::eye(2,2,CV_64FC1);

    if (bIsLeft)
        matTr = matAutoAffR*matRotR.inv()*fScale*matRotL*matAutoAffL.inv(); // ellipse to circle
    else
        matTr = matAutoAffL*matRotL.inv()*fScale*matRotR*matAutoAffR.inv();


    // collect for the return
//    pdAffTR[0] = matTr.at<double>(0,0);
//    pdAffTR[1] = matTr.at<double>(0,1);
//    pdAffTR[2] = matTr.at<double>(1,0);
//    pdAffTR[3] = matTr.at<double>(1,1);
//    cout << matRot.at<double>(0,0) << " " << matRot.at<double>(0,1) << endl;
//    cout << matRot.at<double>(1,0) << " " << matRot.at<double>(1,1) << endl;
//    cout << matAff.at<double>(0,0) << " " << matAff.at<double>(0,1) << endl;
//    cout << matAff.at<double>(1,0) << " " << matAff.at<double>(1,1) << endl;
//    cout << matTr.at<double>(0,0) << " " << matTr.at<double>(0,1) << endl;
//    cout << matTr.at<double>(1,0) << " " << matTr.at<double>(1,1) << endl;

    ///////////////////////////////////
    // crop the image
    ///////////////////////////////////
    float fScaleCropImg = 3.f;
    int nRadPatch = (int) round(nALSCPatchRad * fScaleCropImg);

    Mat matImgCrop = Mat::zeros(2*nRadPatch+1, 2*nRadPatch+1, CV_32FC1);
    matImgCrop.at<float>(nRadPatch, nRadPatch) = getSubPixelValue(ptCen, matImgSrc);

    double pdPosData[2] = {ptCen.x, ptCen.y};
    Mat matPos(2, 1, CV_64FC1, pdPosData);
    Mat matTRPos = Mat::zeros(2,1, CV_64FC1);
    matTRPos = matTr * matPos; // the centre of the patch

    Mat matInvTr = matTr.inv(); // circle to ellipse
    pdInvNorm[0] = matInvTr.at<double>(0,0);
    pdInvNorm[1] = matInvTr.at<double>(0,1);
    pdInvNorm[2] = matInvTr.at<double>(1,0);
    pdInvNorm[3] = matInvTr.at<double>(1,1);

    for (int i = -nRadPatch ; i <= nRadPatch; i++){
            for (int j = -nRadPatch ; j <= nRadPatch; j++){
                if (i == 0 && j == 0 ) continue; // the centre of the patch
                int y = i+nRadPatch; // x position in the patch
                int x = j+nRadPatch;
                double pdData[2] = {(double) j, (double) i};
                Mat matTemp(2,1, CV_64FC1, pdData);
                Mat matNewPos = matInvTr * (matTRPos + matTemp);
                matImgCrop.at<float>(y,x) = getSubPixelValue(Point2f(matNewPos.at<double>(0,0),matNewPos.at<double>(1,0)), matImgSrc);
            }
        }

    return matImgCrop;
}

float CMutualAdapt::getSubPixelValue(const Point2f pt, const Mat& matImg){
    float fRes = 0;
    Rect rectImg(0, 0, matImg.cols, matImg.rows);
    float val1 = 0, val2 = 0, val3 = 0, val4 = 0; // intensity value at four surrounding corners

    int x1 = (int) floor(pt.x);
    int x2 = (int) ceil(pt.x);
    int y2 = (int) ceil(pt.y);
    int y1 = (int) floor(pt.y);

    if (x1 == x2 && y1 == y2){ // when pt.x and pt.y are both integer -> happy case no interpolation
        if (rectImg.contains(Point(x1, y1)))
            fRes = matImg.at<unsigned char>(y1, x1);
        else
            fRes = 0.f;
    }
    else if (x1 == x2 && y1 != y2){ // pt.x is integer but pt.y is not. 1D interpolation is required

        if (rectImg.contains(Point(x1, y1))) val1 = matImg.at<unsigned char>(y1, x1);
        if (rectImg.contains(Point(x1, y2))) val2 = matImg.at<unsigned char>(y2, x1);
        fRes = (val2 - val1) * (pt.y - y1) / (y2 - y1) + val1;
    }
    else if (x1 != x2 && y1 == y2){ // pt.y is integer but pt.x is not. 1D interpolation is required
        if (rectImg.contains(Point(x1, y1))) val1 = matImg.at<unsigned char>(y1, x1);
        if (rectImg.contains(Point(x2, y1))) val2 = matImg.at<unsigned char>(y1, x2);
        fRes = (val2 - val1) * (pt.x - x1) / (x2 - x1) + val1;
    }
    else{ // bidirectional interpolation
        float x = pt.x;
        float y = pt.y;
        if (rectImg.contains(Point(x1, y1))) val1 = matImg.at<unsigned char>(y1, x1);
        if (rectImg.contains(Point(x2, y1))) val2 = matImg.at<unsigned char>(y1, x2);
        if (rectImg.contains(Point(x1, y2))) val3 = matImg.at<unsigned char>(y2, x1);
        if (rectImg.contains(Point(x2, y2))) val4 = matImg.at<unsigned char>(y2, x2);
        fRes = (val1 * (x2 - x) * (y2 - y) / (x2 - x1) / (y2 - y1) +
                val2 * (x - x1) * (y2 - y) / (x2 - x1) / (y2 - y1) +
                val3 * (x2 - x) * (y - y1) / (x2 - x1) / (y2 - y1) +
                val4 * (x - x1) * (y - y1) / (x2 - x1) / (y2 - y1));
    }

    return fRes;
}

bool CMutualAdapt::saveTPResult(string strFile, bool bDistortionParam) const{

    ofstream sfTC;
    sfTC.open(strFile.c_str());

    if (sfTC.is_open()){

        int nSzTC = m_vecTP.size();

        sfTC << nSzTC << " " << 27 << endl; // header

        for (int i = 0; i < nSzTC; i ++){
            CTiePtAdapt tp = m_vecTP.at(i);
            sfTC << tp.m_ptIDPair.x << " " << tp.m_ptIDPair.y << " "
                 << tp.m_ptL.x << " " <<  tp.m_ptL.y << " "
                 << tp.m_ptR.x << " " <<  tp.m_ptR.y << " "
                 << tp.m_fSimVal; // << endl;

            if (bDistortionParam){
                for (int i = 0; i < 4; i++)
                    sfTC << " " << tp.m_pfAffineL[i];
                for (int i = 0; i < 4; i++)
                    sfTC << " " << tp.m_pfAffine[i];
                sfTC << " " << tp.m_ptOffsetL.x << " " <<  tp.m_ptOffsetL.y;
                sfTC << " " << tp.m_ptOffset.x << " " <<  tp.m_ptOffset.y;

//                Mat matDelL = tp.getDelAff(true);
//                Mat matDelR = tp.getDelAff(false);
                for (int m = 0 ; m < 2; m++){
                    for (int n = 0; n < 2; n++){
                        sfTC << " " << tp.m_matALSCAffL.at<double>(m,n);
                    }
                }

                for (int m = 0 ; m < 2; m++){
                    for (int n = 0; n < 2; n++){
                        sfTC << " " << tp.m_matALSCAff.at<double>(m,n);
                    }
                }

            }
            sfTC << endl;
        }
        sfTC.close();
    }
    else
        return false;

    return true;
}

