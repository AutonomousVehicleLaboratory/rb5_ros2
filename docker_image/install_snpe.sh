#!/usr/bin/env bash
set -e
export DEBIAN_FRONTEND=noninteractive

cd /root/install
unzip snpe-1.59.0.3230.zip && cd snpe-1.59.0.3230/lib
export SNPE_TARGET_ARCH=aarch64-ubuntu-gcc7.5
mkdir -p /data/local/tmp/snpeexample/$SNPE_TARGET_ARCH/bin
mkdir -p /data/local/tmp/snpeexample/$SNPE_TARGET_ARCH/lib
mkdir -p /data/local/tmp/snpeexample/dsp/lib
cp -r $SNPE_TARGET_ARCH/*.so /data/local/tmp/snpeexample/$SNPE_TARGET_ARCH/lib
cp -r dsp/*.so  /data/local/tmp/snpeexample/dsp/lib
cd ..
cp -r ./bin/$SNPE_TARGET_ARCH/snpe-net-run /data/local/tmp/snpeexample/$SNPE_TARGET_ARCH/bin

echo 'export SNPE_TARGET_ARCH=aarch64-ubuntu-gcc7.5' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/data/local/tmp/snpeexample/$SNPE_TARGET_ARCH/lib' >> ~/.bashrc
echo 'export PATH=$PATH:/data/local/tmp/snpeexample/$SNPE_TARGET_ARCH/bin' >> ~/.bashrc

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
