#!/bin/bash

ubuntu_version=`lsb_release -rs | sed 's/\.//'`

PYTHONV=3

install_ompl()
{

    cd build/Release
    cmake ../.. -DPYTHON_EXEC=/usr/bin/python${PYTHONV}
    #if [ ! -z $1 ]; then
    make update_bindings
    #fi
    make
    sudo make install
}

install_ompl
