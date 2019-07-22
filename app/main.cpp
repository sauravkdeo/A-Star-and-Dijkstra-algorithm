/**
 * @file      app/main.cpp
 * @brief     solves 8-puzzle using BSF
 * @author    Saurav Kumar
 * @copyright 2019
 *
 **BSD 3-Clause License
 *
 *Copyright (c) 2018, Saurav Kumar
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

#include <../include/map.h>
#include <../include/node.h>
#include <map>
#include <iostream>
#include <algorithm>
#include <queue>
#include <vector>

using std::cout;
using std::endl;
using std::cin;
using std::vector;
using std::priority_queue;

int main() {
  Coord startpoint, endpoint;
  int x, y, u, v, radius, clearance, resolution, opt;

  cout << "Enter 0 for Dijkstra algorithm or 1 for A* Algorithm : ";
  cin >> opt;
  cout << "Robot Radius(Enter 0 for point robot) : ";
  cin >> radius;
  cout << "Clearance (Enter 0 for point robot) : ";
  cin >> clearance;
  cout << "Resolution : ";
  cin >> resolution;
  cout << "Enter the x coordinate of starting point (0 -250) : ";
  cin >> x;
  cout << "Enter the y coordinate of starting point (0 -150) : ";
  cin >> y;
  cout << "Enter the x coordinate of End point (0 -250) : ";
  cin >> u;
  cout << "Enter the y coordinate of End point (0 -150) : ";
  cin >> v;


  startpoint.set_x(x);
  startpoint.set_y(y);
  endpoint.set_x(u);
  endpoint.set_y(v);

  Map map(opt, resolution, radius, clearance);
  if (!map.takeinput(startpoint, endpoint)) {
    return 0;
  }
  Node start;
  start.solve(startpoint, &map);

  cout << "\nCompleted\n";
  cv::waitKey(0);

  return 0;
}
