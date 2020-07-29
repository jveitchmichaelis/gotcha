#!/usr/bin/python3

import os
import numpy as np
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--disparity', type=str,
                        help='Input x-disparity map', required=True)
    parser.add_argument('--ground_truth', type=str,
                        help='Ground truth keypoints', required=True)

    args = parser.parse_args()

    # Load disparity map
    pred_disparity = np.loadtxt(args.disparity, skiprows=1)
    print("Input: {}".format(os.path.abspath(args.disparity)))

    # Calculate disparity from keypoints:
    keypoints = np.loadtxt(args.ground_truth, skiprows=1, delimiter=" ")
    print("Ground truth: {}".format(os.path.abspath(args.ground_truth)))
    gt_xdisp = keypoints[:,0] - keypoints[:,2]
    gt_coords = keypoints[:,:2]

    err = []

    for keypoint in keypoints:
        gt_xdisp = keypoint[2] - keypoint[0]
        x, y = keypoint[:2].astype(int)
        pred_xdisp = pred_disparity[y, x]

        if pred_xdisp == 0:
            continue

        err.append(gt_xdisp - pred_xdisp)

    err = np.array(err)
    print("Note: statistics are computed for supplied keypoints only")
    print("Mean disparity error: {:1.2f} px".format(err.mean()))
    print("Median disparity error: {:1.2f} px".format(np.median(err)))
    print("Stdev disparity error: {:1.2f} px".format(err.std()))
    print("Pixels with err > 0.5px: {:1.2f}%".format(100*sum(np.abs(err) > 0.5)/len(err)))
    print("Err > 1.0: {:1.2f}%".format(100*sum(np.abs(err) > 1.0)/len(err)))
    print("Err > 2.0: {:1.2f}%".format(100*sum(np.abs(err) > 2.0)/len(err)))
    print("Err > 4.0: {:1.2f}%".format(100*sum(np.abs(err) > 4.0)/len(err)))
