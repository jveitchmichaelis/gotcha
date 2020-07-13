#!/usr/bin/env bash

mkdir build
cd build
cmake ..
make -j8
cp ../sift/* .

for TEST in bricks chair corner; do
  ./tpgen ../examples/$TEST/comb_18000_l.png ../examples/$TEST/comb_18000_r.png ../examples/$TEST/tiepoints.txt
  ./gotcha ../examples/$TEST/comb_18000_l.png ../examples/$TEST/comb_18000_r.png ../examples/$TEST/tiepoints.txt ../examples/$TEST
done
