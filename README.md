# Parallel GOTCHA implementation

This repository provides an up-to-date, CPU-parallel, version of the GOTCHA stereo matching algorithm.

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

# License

This code was developed during my PhD between 2012-2016 whilst funded as an STFC-CASE studentship between University College London (UCL) Mullard Space Science Laboratory (MSSL) Imaging Group and Is-Instruments Ltd. It is adapted from [here](https://github.com/mssl-imaging/CASP-GO) which is released under the Apache 2.0 license. This source has been released with permission of the MSSL Imaging Group.

The SIFT executable is provided for historic purposes and the copyright recently expired.

# Citation

If you use this code, please cite my doctoral thesis: https://discovery.ucl.ac.uk/id/eprint/1536083/

and our paper on cloud Mars processing: https://www.sciencedirect.com/science/article/pii/S0032063317303252

```
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