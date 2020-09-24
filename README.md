# Parallel GOTCHA implementation

GOTCHA is a region-growing stereo matcher that uses Adaptive Least Squares Correlation to match image patches. What it lacks in speed it makes up for in accuracy. GOTCHA has been used extensively for Mars terrain reconstruction in various projects. Unlike many matching algorithms, GOTCHA does not place any limits on the range of disparities it can match. The initial stage in the matching process is feature- or tie-point generation - sparse matches between images. The disparity map is then expanded around these regions iteratively.

This repository provides an up-to-date, CPU-parallel, version of the GOTCHA stereo matching algorithm. Compared to its 5th generation predecessor it runs an order of magnitude (or more) faster when single-threaded and offers practically linear speedup with multiple cores. This development enabled matching Gigapixel-scale imagery in a reasonable amount of time (e.g. minutes) rather than days! This speed-up was achieved by making a few modifications to the ALSC implementation, adding parallel execution for the region growing process and making some better choices concerning data structures.

# Building and running

You need a recent version of OpenCV and Eigen3 and they should be discoverable by CMake. OpenCV is used primarily for image IO, so most versions will work, but it should support the newer macro definitions e.g. `cv::COLOR_RGB2GRAY` not `CV_RGB2GRAY`.

Compilation is straightforward on Linux:

``` bash
mkdir build
cd build
cmake ..
make -j8
cp ../sift/* ./
```

and you can then run the test examples. First run `tpgen` to generate and match tiepoints between images, then run `gotcha` to match them:

``` bash
tpgen ../examples/comb_18000_l.png ../examples/comb_18000_r.png tiepoints.txt
gotcha ../examples/comb_18000_l.png ../examples/comb_18000_r.png tiepoints.txt ./
```

you can then view the output disparity and confidence maps. Note that the `disMapX.txt` file is probably what you actually want as it contains floating point disparities. The output PNG images show normalised disparity maps and are provided for diagnostic purposes only (e.g. to see which image regions were matched).

Run the `test.sh` script to build and verify that everything works.

# Notes

There are several parameters that may be worth fiddling with for different image sizes or situations. These are currently hard-coded, but the main parameters are set in `src/gotcha.cpp`:

```
    /* Define GOTCHA parameters */
    CDensifyParam params;
    params.m_nProcType = CDensifyParam::GOTCHA;
    params.m_nTPType = CDensifyParam::TP_UNKNOWN;
    params.m_paramGotcha.m_nNeiType = CGOTCHAParam::NEI_8;
    params.m_paramGotcha.m_paramALSC.m_bIntOffset = 1;
    params.m_paramGotcha.m_paramALSC.m_bWeighting = 0;
    params.m_paramGotcha.m_bNeedInitALSC = 0;
    params.m_paramGotcha.m_paramALSC.m_fEigThr = 130;
    params.m_paramGotcha.m_paramALSC.m_nPatch = 12;
    params.m_paramGotcha.m_paramALSC.m_fAffThr = 1.5;
    params.m_paramGotcha.m_paramALSC.m_fDriftThr = 1.0;
    params.m_paramGotcha.m_paramALSC.m_nMaxIter = 8;
```

For example the `m_fEigThr` parameter roughly controls how confident a match needs to be for it to be accepted. Reducing this will allow more of the image to be matched, but you may get incorrect matches. Conversely, you can raise it and only retain super-confident regions. For most purposes values around 120-140 should be OK. For images such as from Mars where there is generally excellent surface texture in the landscape, you can leave it high.

You can also set the patch size `m_nPatch` - big patch = easier matching, but smoother and potentially wrong disparity maps.

The other parameters `m_fAffThr` and `m_fDriftThr` are how far the patch under consideration is allowed to be warped - these values are in pixels.

`m_bNeedInitALSC` will optionally perform ALSC patch refinement on the original tiepoints, but there isn't a huge benefit in doing this as we run ALSC during the main algorithm anyway.

Some options are not supported - tiling does not exist in this version of the code (for the better, I think) and weighting is not currently implemented.

# License and history of GOTCHA

This source has been released with permission of UCL. This codebase is released under Apache 2.0. Please familiarise yourself with the license file in this repository.

Most of the core routines were either directly taken from, or adapted from, [here](https://github.com/mssl-imaging/CASP-GO) which is released under the **Apache 2.0 license**. To be clear, this codebase represents several generations of MSSL researchers who worked hard on it.

This particular implementation of GOTCHA (also known as Speeded-up or S-GOTCHA) was developed during my PhD between 2012-2016 whilst funded as an STFC-CASE studentship between University College London (UCL) Mullard Space Science Laboratory (MSSL) Imaging Group and Is-Instruments Ltd.

The history of GOTCHA now spans well over 30 years. The origin can be traced back to [Prof. Jan-Peter Muller's](https://www.ucl.ac.uk/mssl/people/prof-jan-peter-muller) involvement in the Alvey MMI-137 project using [transputers](https://en.wikipedia.org/wiki/Transputer) for 2.5D image reconstruction. Since then, development of GOTCHA has been led by Prof. Muller as head of the imaging group at MSSL and several key references can be found below:

``` bibtex
@article{Muller1988RealtimeSM,
  title={Real-time Stereo Matching Spot Using Transputer Arrays},
  author={J. Muller and G. P. Otto and T. Chau and Ka Collins and N. Day and I. Dowman and M. Jackson and M. O'Neill and V. Paramananda and J.B.G. Roberts and A. Stevens and M. Upton},
  journal={International Geoscience and Remote Sensing Symposium, 'Remote Sensing: Moving Toward the 21st Century'.},
  year={1988},
  volume={2},
  pages={1185-1186}
}
```

The GOTCHA algorithm was originally proposed by Gruen, Otto and Chau (hence the acronym):

``` bibtex
@article{Otto1989RegiongrowingAF,
  title={"Region-growing" algorithm for matching of terrain images},
  author={G. P. Otto and T. K. W. Chau},
  journal={Image Vis. Comput.},
  year={1989},
  volume={7},
  pages={83-94}
}
```

While GOTCHA was initially used for processing terrestrial satellite imagery (e.g. SPOT), it has since been used in industrial applications such as surface reconstruction for crack/defect detection and is now routinely used for Martian applications, both for rover and orbital imagery. UCL was the first [NASA Regional Planetary Image Facility](https://en.wikipedia.org/wiki/Regional_Planetary_Image_Facility) (RPIF) outside the US, in 1980, and is therefore extremely well-placed for this kind of research. Prof. Muller is the current director of the UK NASA RPIF at UCL.

Various improvements and modifications have been developed over the years, including Shin and Muller (this paper is already the 5th version of the codebase; s-gotcha is arguably the 6th generation):

``` bibtex
@article{article,
author = {Shin, Dongjoe and Muller, J.-P},
year = {2012},
month = {10},
pages = {3795–3809},
title = {Progressively weighted affine adaptive correlation matching for quasi-dense 3D reconstruction},
volume = {45},
journal = {Pattern Recognition},
doi = {10.1016/j.patcog.2012.03.023}
}
```

also by Shin, Tau and Muller:

``` bibtex
@article{article,
author = {Shin, D.; Tao, Y.; Muller, J.-P.},
year = {2018},
pages = {159-167},
title = {Evaluation of Close-Range Stereo Matching Algorithms Using Stereoscopic Measurements.},
volume = {84},
journal = {Photogramm Eng Rem S},
doi = {10.14358/PERS.84.3.159}
}
```

The [SIFT executable](https://www.cs.ubc.ca/~lowe/keypoints/) is provided for historic purposes and the copyright recently expired. We will move this to OpenCV's implementation shortly.

```
David G. Lowe, "Distinctive image features from scale-invariant keypoints," International Journal of Computer Vision, 60, 2 (2004), pp. 91-110.
```

# Citation

If you use this version of the code, please cite my doctoral thesis: https://discovery.ucl.ac.uk/id/eprint/1536083/

and our paper on cloud Mars processing: https://www.sciencedirect.com/science/article/pii/S0032063317303252

``` bibtex
@article{TAO201830,
title = "Massive stereo-based DTM production for Mars on cloud computers",
journal = "Planetary and Space Science",
volume = "154",
pages = "30 - 58",
year = "2018",
issn = "0032-0633",
doi = "https://doi.org/10.1016/j.pss.2018.02.012",
url = "http://www.sciencedirect.com/science/article/pii/S0032063317303252",
author = "Y. Tao and J.-P. Muller and P. Sidiropoulos and Si-Ting Xiong and A.R.D. Putri and S.H.G. Walter and J. Veitch-Michaelis and V. Yershov",
keywords = "Mars, Global DTM, CTX, HiRISE, CASP-GO, Clouds computing",
}
```
