#!/usr/bin/env bash

mkdir build_release
cd build_release

export CC="ccache $CC"
export CXX="ccache $CXX"

cmake -DCMAKE_INSTALL_PREFIX=${PREFIX} -DPROJECT_INSTALL_DIR=${PREFIX} -DUSE_MKL=ON -DLIBRARY_TYPE=SHARED .. 
make && make test && make install
