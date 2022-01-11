#!/usr/bin/env bash
set -e
export DEBIAN_FRONTEND=noninteractive

cd /usr/src
git clone --single-branch --branch develop https://github.com/borglab/gtsam.git
cd gtsam && mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    ..

make -j4 install && make clean
echo 'export LD_LIBRARY_PATH=/usr/local/lib:LD_LIBRARY_PATH' >> /root/.bashrc


unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*