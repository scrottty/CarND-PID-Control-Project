#!/bin/sh

#  build.sh
#  xcode_project
#
#  Created by Ollie Steiner on 20/08/17.
#  Copyright © 2017 Ollie Steiner. All rights reserved.

rm -rf build
mkdir build
cd build
cmake ..
make
./pid
