
# C++ Dijkstra's and A* algorithm
[![Build Status](https://travis-ci.org/sauravkdeo/Steering-Control-Module.svg?branch=master)](https://travis-ci.org/sauravkdeo/Steering-Control-Module)
[![Coverage Status](https://coveralls.io/repos/github/sauravkdeo/Steering-Control-Module/badge.svg?branch=master)](https://coveralls.io/github/sauravkdeo/Steering-Control-Module?branch=master)
---

## Overview

C++ project to explore node using Dijkstra's or A* algorithm 

###### Main Features

This Project explores a given map using Dijkstra's or A* algorithm.


###### Safety



###### Results

- Fig showing the map exploration using A* and Dijkstra's algorithm.

<p align="center">
<img src="result/dijikstra.gif" width="400" height="270" border="10">
<img src="result/a_star.gif" width="400" height="270" border="10">
</p>

## License

BSD 3-Clause License

Copyright (c) 2018, Saurav
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Dependencies
- Google Mock
- CMake version at least 3.2
- Google Test

## Standard install via command-line and program run
```
unzip the files to a location
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run program: ./app/Project2
```

## Generate doxygen documentation


```
cd <path to repository>
cd docs
doxygen steer_doxy.Doxyfile
```

## Building for code coverage
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.
