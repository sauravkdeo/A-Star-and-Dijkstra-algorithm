 /**
  * @file      include/SteeringControl.h
  * @brief     Header file for declaring the class SteeringControl,its attributes and its methods
  *            along with class SteeringControlOutput, class Coord and their data members
  * @author    Saurav Kumar
  * @copyright 2019
  *
  **BSD 3-Clause License
  *
  *Copyright (c) 2019, Saurav Kumar
  *All rights reserved.
  *
  *Redistribution and use in source and binary forms, with or without
  *modification, are permitted provided that the following conditions are met:
  *
  * Redistributions of source code must retain the above copyright notice, this
  * list of conditions and the following disclaimer.
  *
  * Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  *  and/or other materials provided with the distribution.
  *
  * Neither the name of the copyright holder nor the names of its
  *  contributors may be used to endorse or promote products derived from
  *  this software without specific prior written permission.
  *
  *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */
#include <math.h>
#include <../include/coord.h>
#include <algorithm>
#include <iostream>
/** constructor**/
/*Coord::Coord() {
  X = 0;
  Y = 0;
}
* destructor*
Coord::~Coord() {
}*/
int Coord::get_x() {
  return X;
}

void Coord::set_x(int x_coordinate_value) {
  X = x_coordinate_value;
}
int Coord::get_y() {
  return Y;
}

void Coord::set_y(int y_coordinate_value) {
  Y = y_coordinate_value;
}
