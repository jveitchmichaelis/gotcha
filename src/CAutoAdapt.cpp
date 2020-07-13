#include "CAutoAdapt.h"

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

void printProgress (double percentage)
{
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf ("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush (stdout);
}

CAutoAdapt::CAutoAdapt(string strImg, string strIP){
    m_matImgSrc = imread(strImg, 0);

    vector<KeyPoint>* pvecIP =  new vector<KeyPoint>;
    m_bDeleteIP = true;

    ifstream sfFrame;
    sfFrame.open(strIP.c_str());
    if (sfFrame.is_open()){

        int nLine = 0;
        int nEle = 0;
        sfFrame >> nLine;
        sfFrame >> nEle;
        // collect data
        int nCount = 0;
        while (nCount < nLine){
            KeyPoint kp;
            sfFrame >> kp.pt.x >> kp.pt.y >> kp.size >> kp.angle >> kp.class_id; // get key poiints
            pvecIP->push_back(kp);
            nCount++;
        }
        //pvecIP->pop_back();
        sfFrame.close();
    }

    m_pvecIP = pvecIP;
    m_pdAff = NULL;
}

CAutoAdapt::~CAutoAdapt(){
    clearOutputBuffer();
    if (m_bDeleteIP) delete m_pvecIP;
}

void CAutoAdapt::clearOutputBuffer(){
    if (isDataReady()){
        delete [] m_pdAff;
        m_pdAff = NULL;
    }
}

Mat CAutoAdapt::getAffineParam(int nPos) const{
    Mat matRes = Mat::zeros(2,2,CV_64FC1);

    matRes.at<double>(0,0) = m_pdAff[nPos*4];
    matRes.at<double>(0,1) = m_pdAff[nPos*4+1];
    matRes.at<double>(1,0) = m_pdAff[nPos*4+2];
    matRes.at<double>(1,1) = m_pdAff[nPos*4+3];

    return matRes;
}


bool CAutoAdapt::saveAffineEstimation(string strFile) const{
    // you can draw the reuslt using MATLAB

    ofstream sfAff;
    sfAff.open(strFile.c_str());

    if (sfAff.is_open()){
        int nSzIP = m_pvecIP->size();

        sfAff << nSzIP << " " << 7 << endl; //header

        for (int i = 0; i < nSzIP; i ++){
            KeyPoint kp = m_pvecIP->at(i);

            sfAff << kp.pt.x << " " << kp.pt.y << " "<< kp.size;

            for (int j = 0; j < 4; j++)
                sfAff << " " << m_pdAff[i*4+j];
            sfAff << endl;
        }
        sfAff.close();
     }
    else
        return false;

    return true;
}

void CAutoAdapt::doAutoAdaptation(const int nMaxIter, const double dThrCicle, const double dThrDist){

    //////////////////////////////////////////////
    // processing parameters
    //////////////////////////////////////////////
    // clear output buffer
    clearOutputBuffer();
    int nIP = m_pvecIP->size();
    m_pdAff = new double [nIP*4];

    ////////////////////////////////////////////////
    // iteration for self-adaptation
    ////////////////////////////////////////////////
    for (int j = 0 ; j < nIP; j++) {    // for every IP points

        Mat matUpdatedAff = Mat::eye(2,2,CV_64FC1);
        Mat matUpdatedAffinv = Mat::eye(2,2,CV_64FC1);
        bool bFoundSolution = false;

        printProgress(100*j/nIP);
        std::cout << j << std::endl;

        int i =0;
        for ( i = 0; i < nMaxIter; i++){

            // 2x2 Explicit inverse
            double a = matUpdatedAff.at<double>(0,0);
            double b = matUpdatedAff.at<double>(0,1);
            double c = matUpdatedAff.at<double>(1,1);
            double d = matUpdatedAff.at<double>(1,1);

            double scale = 1.0/(a*d-b*c);

            matUpdatedAffinv.at<double>(0,0) = d;
            matUpdatedAffinv.at<double>(0,1) = -b;
            matUpdatedAffinv.at<double>(1,0) = -c;
            matUpdatedAffinv.at<double>(1,1) = a;

            matUpdatedAffinv *= scale;

            Mat matRDesc = getHarrisRegionDesc(j, matUpdatedAffinv); // get region descriptor from a transformed image (2-by-2 symmetric mat)
//                         cout << matRDesc.at<double>(0,0) << " " << matRDesc.at<double>(0,1) << endl
//                                        << matRDesc.at<double>(1,0) << " " << matRDesc.at<double>(1,1) << endl << endl;

            Mat matAff = getAffineNormalisingTR(matRDesc);        // M^(-0.5) which change a circle to an ellipse

//                        cout << matAff.at<double>(0,0) << " " << matAff.at<double>(0,1) << endl
//                             << matAff.at<double>(1,0) << " " << matAff.at<double>(1,1) << endl << endl;
            matUpdatedAff = matAff * matUpdatedAff;               // accumulate the distortion. This is a symmetric matrix
//                        cout << matUpdatedAff.at<double>(0,0) << " " << matUpdatedAff.at<double>(0,1) << endl
//                             << matUpdatedAff.at<double>(1,0) << " " << matUpdatedAff.at<double>(1,1) << endl << endl;

            ////////////////////////////////
            // test the stop conditions
            ////////////////////////////////            
            // debug
            SVD svdTest(matAff);

            // debug
//            cout << matAff.at<double>(0,0) << " " << matAff.at<double>(0,1) << endl
//                 << matAff.at<double>(1,0) << " " << matAff.at<double>(1,1) << endl << endl;
//            cout << svdTest.u.at<double>(0,0) << " " << svdTest.u.at<double>(0,1) << endl
//                 << svdTest.u.at<double>(1,0) << " " << svdTest.u.at<double>(1,1) << endl << endl;
//            cout << svdTest.w.at<double>(0,0) << " " << svdTest.w.at<double>(0,1) << endl
//                 << svdTest.w.at<double>(1,0) << " " << svdTest.w.at<double>(1,1) << endl << endl;
//            cout << svdTest.vt.at<double>(0,0) << " " << svdTest.vt.at<double>(0,1) << endl
//                 << svdTest.vt.at<double>(1,0) << " " << svdTest.vt.at<double>(1,1) << endl << endl;

            // stop condition 1 from Krystian's paper:
            // If an updating TR is almost identity matrix -> the change is small -> stop adapting
            double dCost = 1 - (svdTest.w.at<double>(1,0) / svdTest.w.at<double>(0,0));
            if ( dCost <  dThrCicle){
                bFoundSolution = true;
                break;
            }

            // stop condition 2 from Krystian's paper: when the distortion is too significant, stop changing the shape
            // FYI, in the Frederik's paper, it is implemented as that no point in the region can move by 0.5 pixel after the transformation
            svdTest(matUpdatedAff);
            // debug
//            cout << matUpdatedAff.at<double>(0,0) << " " << matUpdatedAff.at<double>(0,1) << endl
//                 << matUpdatedAff.at<double>(1,0) << " " << matUpdatedAff.at<double>(1,1) << endl << endl;
//            cout << svdTest.u.at<double>(0,0) << " " << svdTest.u.at<double>(0,1) << endl
//                 << svdTest.u.at<double>(1,0) << " " << svdTest.u.at<double>(1,1) << endl << endl;
//            cout << svdTest.w.at<double>(0,0) << " " << svdTest.w.at<double>(0,1) << endl
//                 << svdTest.w.at<double>(1,0) << " " << svdTest.w.at<double>(1,1) << endl << endl;
//            cout << svdTest.vt.at<double>(0,0) << " " << svdTest.vt.at<double>(0,1) << endl
//                 << svdTest.vt.at<double>(1,0) << " " << svdTest.vt.at<double>(1,1) << endl << endl;

            dCost = svdTest.w.at<double>(0,0) / svdTest.w.at<double>(1,0);
            if ( dCost > dThrDist){
                bFoundSolution = true;
                break;
            }
        }

        // debug
//        cout << "ID: " << j << " at " << i << endl;

        if (bFoundSolution) {
//             update the affine parameters
            m_pdAff[4*j] = matUpdatedAff.at<double>(0,0);
            m_pdAff[4*j+1] = matUpdatedAff.at<double>(0,1);
            m_pdAff[4*j+2] = matUpdatedAff.at<double>(1,0);
            m_pdAff[4*j+3] = matUpdatedAff.at<double>(1,1);
        }
        else{
            m_pdAff[j*4] = 1.0;
            m_pdAff[j*4+1] = 0;
            m_pdAff[j*4+2] = 0;
            m_pdAff[j*4+3] = 1.0;
        }
    }
}

Mat CAutoAdapt::getAffineNormalisingTR(const Mat& matRDesc){
    Mat matRes = Mat::eye(2,2,CV_64FC1);

//    cout << matRDesc.at<double>(0,0) << " " << matRDesc.at<double>(0,1) << endl
//         << matRDesc.at<double>(1,0) << " " << matRDesc.at<double>(1,1) << endl << endl;

    Mat matEigVal = Mat::zeros(2,2,CV_64FC1);
    Mat matEigVec = Mat::zeros(2,2,CV_64FC1);
    eigen(matRDesc, matEigVal, matEigVec);  // matRDesc = matEigVec.t() * matEigVal * matEigVec;

//    cout << matEigVal.at<double>(0,0) << " " << matEigVal.at<double>(0,1) << endl
//         << matEigVal.at<double>(1,0) << " " << matEigVal.at<double>(1,1) << endl << endl;

//    cout << matEigVec.at<double>(0,0) << " " << matEigVec.at<double>(0,1) << endl
//         << matEigVec.at<double>(1,0) << " " << matEigVec.at<double>(1,1) << endl << endl;


//    if(matEigVal.at<double>(0,0) < 0){
//        matEigVec.at<double>(0,0) = -matEigVec.at<double>(0,0);
//        matEigVec.at<double>(0,1) = -matEigVec.at<double>(0,1);
//        matEigVal.at<double>(0,0) = fabs(matEigVal.at<double>(0,0));
//    }
//    if(matEigVal.at<double>(0,1) < 0){
//        matEigVec.at<double>(1,0) = -matEigVec.at<double>(1,0);
//        matEigVec.at<double>(1,1) = -matEigVec.at<double>(1,1);
//        matEigVal.at<double>(0,1) = fabs(matEigVal.at<double>(0,1));
//    }

//    cout << matEigVal.at<double>(0,0) << " " << matEigVal.at<double>(0,1) << endl
//         << matEigVal.at<double>(1,0) << " " << matEigVal.at<double>(1,1) << endl << endl;

//    cout << matEigVec.at<double>(0,0) << " " << matEigVec.at<double>(0,1) << endl
//         << matEigVec.at<double>(1,0) << " " << matEigVec.at<double>(1,1) << endl << endl;


    double pdData[4] = {1, 0, 0, sqrt(matEigVal.at<double>(0,1))/sqrt(matEigVal.at<double>(0,0))}; // for normalising
    Mat matSQD (2, 2, CV_64FC1, pdData);
    Mat matTemp = Mat::zeros(2, 2, CV_64FC1);
    matTemp = matEigVec.t()* matSQD *matEigVec;//.t().inv() don't needed as it is orthogonal; // matRDesc = matTemp.t() * matTemp

//    cout << matTemp.at<double>(0,0) << " " << matTemp.at<double>(0,1) << endl
//         << matTemp.at<double>(1,0) << " " << matTemp.at<double>(1,1) << endl << endl;

    matRes = matTemp; // / matTemp.at<double>(1,1); // normalisation -> is it required?

    return matRes.inv();

    ////////////////////////////////////////////////////////////////////////////////////
    // Matlab code
    ////////////////////////////////////////////////////////////////////////////////////
//    [V D] = eig(M);
//    % theta = atan2(V(4),V(3));
//    % ellipse(1/sqrt(D(1)), 1/sqrt(D(4)), theta, Fl(1,571), Fl(2,571));
//    % plot(Fl(1,571),Fl(2,571), 'rx');
//    % hold off;
//    % Normalise
//    % A'*M2*A = M
//    % inv(A')M inv(A) = M2
//    A = V*[sqrt(D(1)), 0; 0, sqrt(D(4))]*inv(V); % * V';
//    % do I need to renormalise Affine parameters??
//    A = A/A(end);
}

float CAutoAdapt::getSubPixelValue(const Point2f pt, const Mat& matImg){
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

Mat CAutoAdapt::getHarrisRegionDesc(const int nIPPos, const Mat& matAff){

    // Before distorting an image using the affine parameters,
    // it is needed to crop the image first.
    // The radius of the cropped image is defined as (1+2*fScaleKernel)*fSigma
    // because we need some boundary for the Gaussian convolution later

    //////////////////////////////
    // I. distort image
    //////////////////////////////

    // I.1. define the size of a distortion patch
    // The Matlab code transfoms the whole input image and cropped.
    // However, it is not an option if an input image is huge!
    KeyPoint kpSrc = m_pvecIP->at(nIPPos);

    // debug
//    kpSrc.pt = Point2f(529.142, 371.884);
//    kpSrc.size = 20;


    float fSigma = kpSrc.size;
    float fScaleKernel = 2.5f;                      // 3 sigma would be more correct but slow the convolution process
    int nRad = (int) round( (1+2*fScaleKernel)*fSigma ) + 1; // the radius of a cropped patch
                                                             // +1 is required as Gradient will not be computed at the boudary

    // I.2. crop an image patch
    Mat matImgCrop = Mat::zeros(2*nRad+1, 2*nRad+1, CV_32FC1);
    matImgCrop.at<float>(nRad, nRad) = getSubPixelValue(kpSrc.pt, m_matImgSrc);

    double pdPosData[2] = {kpSrc.pt.x, kpSrc.pt.y};
    Mat matPos(2, 1, CV_64FC1, pdPosData);
    Mat matTRPos = Mat::zeros(2,1, CV_64FC1);
    matTRPos = matAff * matPos;

    // debug
//    cout << matAff.at<double>(0,0) << " " <<  matAff.at<double>(0,1) << " " <<  matAff.at<double>(1,0) << " " <<  matAff.at<double>(1,1) << " "
//         << matPos.at<double>(0,0) << " " << matPos.at<double>(1,0) << endl;
//    cout << matTRPos.at<double>(0,0) << " " << matTRPos.at<double>(1,0) << endl;
    // end debug

    Mat matInvAff = matAff.inv();

//    cout << matInvAff.at<double>(0,0) << " " << matInvAff.at<double>(0,1) << endl
//         << matInvAff.at<double>(1,0) << " " << matInvAff.at<double>(1,1) << endl;
//    cout << endl;

    // I.3. colect data
    for (int i = -nRad ; i <= nRad; i++){
        for (int j = -nRad ; j <= nRad; j++){
            if (i == 0 && j == 0 ) continue; // the centre of the patch
            int y = i+nRad;
            int x = j+nRad;            
            double pdData[2] = {(double) j, (double) i};
            Mat matTemp(2,1, CV_64FC1, pdData);
            Mat matNewPos = matInvAff * (matTRPos + matTemp);
            matImgCrop.at<float>(y,x) = getSubPixelValue(Point2f(matNewPos.at<double>(0,0),matNewPos.at<double>(1,0)), m_matImgSrc);
        }
    }

    // debug
//    for(int i = 0 ; i < matImgCrop.rows; i++){
//        for (int j = 0 ; j < matImgCrop.cols; j++){
//            cout << matImgCrop.at<float>(i,j) << " ";
//        }
//        cout << endl;
//    }
//    cout << "======================="<<endl;
    //////////////////////////////
    // II. get Gradient of image
    //////////////////////////////
    int nWPatch = 2*nRad+1;
    Size szImgPatch(nWPatch, nWPatch);
    float* pfGx = new float [szImgPatch.area()];
    float* pfGy = new float [szImgPatch.area()];
    float* pfGxy = new float [szImgPatch.area()];

    getGradient(matImgCrop, pfGx, true);
    getGradient(matImgCrop, pfGy, false);


    //debug
//    ofstream sfAff;
//    sfAff.open("/Users/DShin/Desktop/gradX.txt");
//    if (sfAff.is_open()){
//        for(int i = 0 ; i < matImgCrop.rows; i++){
//            for (int j = 0 ; j < matImgCrop.cols; j++){
//                sfAff << pfGx[i*matImgCrop.cols+j] << " ";
//            }
//            sfAff << endl;
//        }
//        sfAff.close();
//     }
//    cout << "======================="<<endl;
//    for(int i = 0 ; i < matImgCrop.rows; i++){
//        for (int j = 0 ; j < matImgCrop.cols; j++){
//            cout << pfGy[i*matImgCrop.cols+j] << " ";
//        }
//        cout << endl;
//    }
//    cout << "======================="<<endl;


    for (int i = 0 ; i < nWPatch*nWPatch; i++){
        pfGxy[i] = pfGx[i] * pfGy[i];
        pfGx[i] = pfGx[i] * pfGx[i];
        pfGy[i] = pfGy[i] * pfGy[i];
    }


    // Gaussian convolution
    int nWKer = 2*(int)round(fScaleKernel*fSigma)+1;
    Size szKernel(nWKer, nWKer);
    getGaussianBlur(pfGx, szImgPatch, szKernel, fSigma); // pfGx will be changed after calling this function,
    getGaussianBlur(pfGy, szImgPatch, szKernel, fSigma); // i.e., source will be used as destination parameter
    getGaussianBlur(pfGxy, szImgPatch, szKernel, fSigma);

    //debug
    //ofstream sfAff;
//    sfAff.open("/Users/DShin/Desktop/blur.txt");
//    if (sfAff.is_open()){
//        for(int i = 0 ; i < matImgCrop.rows; i++){
//            for (int j = 0 ; j < matImgCrop.cols; j++){
//                sfAff << pfGx[i*matImgCrop.cols+j] << " ";
//            }
//            sfAff << endl;
//        }
//        sfAff.close();
//     }
//    cout << "======================="<<endl;
//



    /////////////////////////////////
    // III. get region descriptor, M
    /////////////////////////////////
    Mat matRes = Mat::eye(2,2,CV_64FC1);
    float a = 0, b = 0, c =0;
    int nRad2 = (int) round(fSigma);

    int nCount = 0 ;
    for (int i = -nRad2; i <= nRad2; i++){
        for (int j = -nRad2; j <= nRad2; j++){
            int y = nRad + i;
            int x = nRad + j;
            a += pfGx[y*nWPatch+x];
            b += pfGxy[y*nWPatch+x];
            c += pfGy[y*nWPatch+x];

            nCount++;
        }
    }

    matRes.at<double>(0,0) = a / nCount;//(1.f+2.f*nRad2);
    matRes.at<double>(1,0) = b / nCount;//(1.f+2.f*nRad2);
    matRes.at<double>(0,1) = b / nCount;//(1.f+2.f*nRad2);
    matRes.at<double>(1,1) = c / nCount;//(1.f+2.f*nRad2);

    //////////////////////////////
    // clean-up
    //////////////////////////////
    delete [] pfGx;
    delete [] pfGy;
    delete [] pfGxy;

    return matRes;

    ////////////////////////////////////////////////////////////////////////////////////
    // Matlab code
    ////////////////////////////////////////////////////////////////////////////////////
    //    if nargin < 3
    //        Aff = eye(2);
    //    end
    //    % get an affine-normalised an image
    //    % (NB. MATLAB ueses different affine notation,
    //    % x' = xA rather than x' = Ax used in general)
    //    t_aff = maketform('affine', [double(Aff') [0 0]'; 0 0 1]);
    //    %t_aff = maketform('affine', inv(Aff));
    //    % NB. 'imtransform' function automatically shifts
    //    % the origin of your output image to make as much of the transformed image
    //    % visible as possible.
    //    [imgAff xOff yOff]= imtransform(img, t_aff,'XYScale',1);
    //    Pos = tformfwd(t_aff, Fl(1), Fl(2));
    //    Pos(1) = Pos(1) - xOff(1) + 1;
    //    Pos(2) = Pos(2) - yOff(1) + 1;

    //    [Ix,Iy] = gradient(imgAff);

    //    % Gaussien Filter
    //    g = fspecial('gaussian', round(Fl(3)*2.5), Fl(3)); % this is an integration filter
    //    Ix2 = conv2(Ix.^2, g, 'same');
    //    Iy2 = conv2(Iy.^2, g, 'same');
    //    Ixy = conv2(Ix.*Iy, g,'same');

    //    % get region decriptor
    //    % M = [interp2(Ix2, Pos(1),Pos(2)) interp2(Ixy, Pos(1),Pos(2))
    //    %      interp2(Ixy, Pos(1),Pos(2)) interp2(Iy2, Pos(1),Pos(2))];

    //    % use average sum
    //    idxYStart = max(1, uint64(round(Pos(2)-Fl(3))));
    //    idxYEnd = min(size(imgAff,1), uint64(round(Pos(2)+Fl(3))));
    //    idxXStart = max(1, uint64(round(Pos(1)-Fl(3))));
    //    idxXEnd = min(size(imgAff,2), uint64(round(Pos(1)+Fl(3))));

    //    a = sum(sum(Ix2(idxYStart:idxYEnd, idxXStart:idxXEnd)));
    //    b = sum(sum(Ixy(idxYStart:idxYEnd, idxXStart:idxXEnd)));
    //    c = sum(sum(Iy2(idxYStart:idxYEnd, idxXStart:idxXEnd)));

    //    sz = size(idxYStart:idxYEnd, 2) * size(idxXStart:idxXEnd, 2);
    //    M = [a b;
    //         b c]/sz;
    //    end
    ////////////////////////////////////////////////////////////////////////////////////
}


void CAutoAdapt::getGradient(const Mat& matSrc, float* pfG, bool bIsGx) {

    int nW = matSrc.cols;
    int nH = matSrc.rows;

    // clear buffer
    for (int i = 0; i < nH; i++){
        for (int j = 0 ; j < nW; j++){
               *(pfG + j + nW*i) = 0.f;
        }
    }

    // get gradient
    for (int j = 1; j < nH - 1; j++) {
        for (int i = 1; i < nW - 1; i++) {

            double dVal1 = 0, dVal2 = 0;
            dVal1 = matSrc.at<float>(j, i);

            if (bIsGx) dVal2 = matSrc.at<float>(j, i+1);
            else dVal2 = matSrc.at<float>(j+1, i);

            *(pfG + i + nW*j) = dVal2 - dVal1;
        }
    }

//        //debug

//    ofstream sfAff;
//    sfAff.open("/Users/DShin/Desktop/tst.txt");

//    if (sfAff.is_open()){
//            for(int i = 0 ; i < matSrc.rows; i++){
//                for (int j = 0 ; j < matSrc.cols; j++){
//                    sfAff << pfG[i*matSrc.cols+j] << " ";
//                }
//                sfAff << endl;
//            }

//        sfAff.close();
//     }

}

void CAutoAdapt::getGaussianBlur(float* pfImgData, const Size szImg, Size szKernel, float fSigma){

    // make src mat
    Mat matSrc(szImg, CV_32FC1, pfImgData); // header only
    Mat matDst = Mat::zeros(szImg, CV_32FC1);

  //  Mat matGausKer = getGaussianKernel(szKernel.height, fSigma);

    // debug
//    for (int i = 0; i < szImg.height; i++){
//        for(int j = 0; j < szImg.width; j++)
//            cout << matGausKer.at<double>(i,j) << " ";
//        cout << endl;
//    }

    GaussianBlur(matSrc, matDst, szKernel, fSigma, fSigma, BORDER_CONSTANT);

    // collect data to buffer
    for (int i = 0; i < szImg.height; i++){
        for (int j = 0; j < szImg.width; j++){
            pfImgData [i*szImg.width+j] = matDst.at<float>(i,j);
        }
    }
}

