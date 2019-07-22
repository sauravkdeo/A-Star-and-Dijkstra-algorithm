
/**
 * @file      include/map.cpp
 * @brief     cpp file for building map
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



#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <../include/map.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <vector>


using std::ofstream;

Map::Map(int choice, int resol, int robotRadius,
         int Clearance):
                    option(choice), resolution(resol),
                    robotradius(robotRadius), clearance(Clearance) {
  gridmapheight = 151;
  gridmapwidth = 251;

  gridmap.resize(gridmapwidth);
  for (int i = 0; i < gridmapwidth; i++) {
    gridmap[i].resize(gridmapheight);
  }

  for (int i = 0; i < gridmapwidth; i++) {
    for (int j = 0; j < gridmapheight; j++) {
      gridmap[i][j]= 1;
    }
  }
  frame = cv::Mat(gridmapheight, gridmapwidth,
                  CV_8UC3, cv::Scalar(255, 255, 255));
  buildmap();
}
Map::~Map() {
}
void Map::definepolygonobstacle(int n, vector<vector<int>> A) {
  if (n > 2) {
    vector<vector<double>> B(n, std::vector<double>(3));
    double theeta, slope_theeta;
    int count;
    double c;
    for (int i = 0; i <= (n-1); i++) {
      if (i < (n-1)) {
        theeta = atan2((A[i+1][1]-A[i][1]), (A[i][0]-A[i+1][0]));
        slope_theeta =  atan2((A[i+1][1]-A[i][1]), (A[i+1][0]-A[i][0]));
        c = (sin(theeta)* A[i][0]) + (cos(theeta) * A[i][1]);
      }
      if (i == (n-1)) {
        theeta = atan2((A[0][1]-A[i][1]), (A[i][0]-A[0][0]));
        slope_theeta =  atan2((A[0][1]-A[i][1]), (A[0][0]-A[i][0]));
        c = (sin(theeta)* A[i][0]) + (cos(theeta) * A[i][1]);
      }
      B[i][0] = theeta;
      B[i][1] = c;
      B[i][2] = slope_theeta;
    }

    for (int j = 0; j < gridmapheight; j++) {
      for (int i = 0; i < gridmapwidth; i++) {
        count = 0;
        for (int k = 0; k <= n-1; k++) {
          if ((sin(B[k][0]) * i+cos(B[k][0]) * j)-B[k][1] <= 0) {
            count = count+1;
          }
          if (count == n) {
            gridmap[i][j]= -1;
          }
        }
      }
    }
    formobstacleboundary();
    joinconcaveshapes();
  }
}

void Map::definecurveobstacle(Coord center, int a, int b) {
  for (int j = 0; j < gridmapheight; j++) {
    for (int i = 0; i < gridmapwidth; i++) {
      if (gridmap[i][j] > 0) {
        if (((i-center.get_x())*(i-center.get_x())*(b*b))
            +((j-center.get_y())*(j-center.get_y())*(a*a))-(a*a*b*b) <= 0) {
          gridmap[i][j] = -1;
        }
      }
    }
  }
  formobstacleboundary();
}

void Map::addobstaclebuffer() {
  int buffer = robotradius + clearance;
  for (int j = 0; j < gridmapheight; j++) {
    for (int i = 0; i < gridmapwidth; i++) {
      if (gridmap[i][j] == 0) {
        for (int k = 0; k < gridmapheight; k++) {
          for (int l = 0; l < gridmapwidth; l++) {
            if (((l-i)*(l-i)) + ((k-j)*(k-j))-(buffer*buffer) <= 0) {
              if (gridmap[l][k] > 0) {
                gridmap[l][k] = -1;
              }
            }
          }
        }
      }
    }
  }
}



int Map::getgridmapheight() {
  return gridmapheight;
}


int Map::getgridmapwidth() {
  return gridmapwidth;
}

void Map::setgridmap(int x, int y, int value) {
  gridmap[x][y] = value;
}
int Map::getgridmap(int x, int y) {
  return gridmap[x][y];
}

void Map::formobstacleboundary() {
  for (int j = 0; j < gridmapheight; j++) {
    for (int i = 0; i < gridmapwidth; i++) {
      if (gridmap[i][j] == -1) {
        if (gridmap[i][j+1]== 1 || gridmap[i+1][j+1]== 1 ||
            gridmap[i+1][j]== 1 || gridmap[i+1][j-1]== 1 ||
            gridmap[i][j-1]== 1 || gridmap[i-1][j-1]== 1 ||
            gridmap[i-1][j]== 1 || gridmap[i-1][j+1]== 1) {
          gridmap[i][j]= 0;
        }
      }
    }
  }
}

void Map::joinconcaveshapes() {
  for (int j = 0; j < gridmapheight; j++) {
    for (int i = 0; i < gridmapwidth; i++) {
      if (gridmap[i][j] == 0) {
        if (gridmap[i][j+1] <= 0 && gridmap[i+1][j+1] <= 0 &&
            gridmap[i+1][j] <= 0 && gridmap[i+1][j-1] <= 0 &&
            gridmap[i][j-1] <= 0 && gridmap[i-1][j-1] <= 0 &&
            gridmap[i-1][j] <= 0 && gridmap[i-1][j+1] <= 0) {
          gridmap[i][j]= -1;
        }
      }
    }
  }
}
void Map::buildmap() {
  int hexedge1l, hexedge2l, hexedge3l, hexedge4l, hexedge5l, hexedge6l;
  int hexedge1r, hexedge2r, hexedge3r, hexedge4r, hexedge5r, hexedge6r;
  int rectedge1l, rectedge2l, rectedge3l, rectedge4l;
  int rectedge1r, rectedge2r, rectedge3r, rectedge4r;
  hexedge1l = 170;
  hexedge1r = 90;

  hexedge2l = 163;
  hexedge2r = 52;

  hexedge3l = 125;
  hexedge3r = 56;

  hexedge4l = 150;
  hexedge4r = 15;

  hexedge5l = 173;
  hexedge5r = 15;

  hexedge6l = 193;
  hexedge6r = 52;

  vector<vector<int>> hexaobstacle1
  {{hexedge1l, hexedge1r},
    {hexedge2l, hexedge2r},
    {hexedge5l, hexedge5r},
    {hexedge6l, hexedge6r}};

  vector<vector<int>> hexaobstacle2
  {{hexedge2l, hexedge2r},
    {hexedge3l, hexedge3r},
    {hexedge4l, hexedge4r},
    {hexedge5l, hexedge5r}};

  rectedge1l = 50;
  rectedge1r = 67.5;
  rectedge2l = 100;
  rectedge2r = 67.5;
  rectedge3l = 100;
  rectedge3r = 112.5;
  rectedge4l = 50;
  rectedge4r = 112.5;


  vector<vector<int>> rectangleobstacle
  {{rectedge1l, rectedge1r},
    {rectedge2l, rectedge2r},
    {rectedge3l, rectedge3r},
    {rectedge4l, rectedge4r}};

  Coord circle;
  Coord eclipse;
  circle.set_x(190);
  circle.set_y(130);
  eclipse.set_x(140);
  eclipse.set_y(120);

  definepolygonobstacle(4, hexaobstacle1);
  definepolygonobstacle(4, hexaobstacle2);

  definepolygonobstacle(4, rectangleobstacle);

  definecurveobstacle(circle, 15, 15);
  definecurveobstacle(eclipse, 15, 6);

  addobstaclebuffer();
  //
  seemap();
}

Coord Map::getendcoord() {
  return endCoord;
}
Coord Map::getstartingCoord() {
  return startingCoord;
}

void Map::seemap() {
  for (int i = 0; i < gridmapheight; i++) {
    for (int j = 0; j < gridmapwidth; j++) {
      if (gridmap[j][gridmapheight-i] == -1) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
      }
      if (gridmap[j][gridmapheight-i] == 0) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
      }
      if (gridmap[j][gridmapheight-i] == 2) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255,  0);
      }
      if (gridmap[j][gridmapheight-i] == 3) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
      }
      if (gridmap[j][gridmapheight-i] == 4) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
      }
      if (gridmap[j][gridmapheight-i] == 5) {
        frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
      }
    }
  }
  cv:: Mat outImg;
  cv::resize(frame, outImg, cv::Size(), 4, 4);
  cv::namedWindow("Project2", cv::WINDOW_AUTOSIZE);
  cv::imshow("Project2", outImg);
}

void Map::updatemap(Coord from, Coord to, int fillvalue) {
  Coord fillposition;
  double deltax = (to.get_x()-from.get_x());
  double deltay = (to.get_y()-from.get_y());
  double maximum = std::max(abs(deltax), abs(deltay));
  deltax = (to.get_x()-from.get_x())/maximum;
  deltay = (to.get_y()-from.get_y())/maximum;
  for (int i = 0; i < maximum; i++) {
    fillposition.set_x(from.get_x()+ceil(i*deltax));
    fillposition.set_y(from.get_y()+ceil(i*deltay));
    fillmap(fillposition, fillvalue);
  }
}


void Map::fillmap(Coord point, int value) {
  int i = gridmapheight-point.get_y()-1;
  int j = point.get_x();
  gridmap[j][gridmapheight-i-1] = value;
  if (value == -1) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
  }
  if (value == 0) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
  }
  if (value == 2) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255,  0);
  }
  if (value == 3) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
  }
  if (value == 4) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
  }
  if (value == 5) {
    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
  }
  cv:: Mat outImg1;
  cv::resize(frame, outImg1, cv::Size(), 4, 4);
  cv::waitKey(1);
  cv::imshow("Project2", outImg1);
}

int Map::getoption() {
  return option;
}

int Map::getresolution() {
  return resolution;
}

void Map::setresolution(int resol) {
  resolution = resol;
}
bool Map::checkboundary(Coord point) {
  return (point.get_x() >= 0 && point.get_x() < gridmapwidth &&
      point.get_y() >= 0 && point.get_y() < gridmapheight);
}
bool Map::checksanity(Coord point) {
  return(gridmap[point.get_x()][point.get_y()] == 1 && checkboundary(point));
}

bool Map::takeinput(Coord sp, Coord ep) {
  int sanity = 0;
  if (checksanity(sp)) {
    startingCoord = sp;
    fillmap(startingCoord, 2);
    sanity = sanity+1;
  } else {
    std::cout << "Invalid Starting Point!!\n";
  }
  if (checksanity(ep)) {
    endCoord = ep;
    fillmap(endCoord, 4);
    sanity = sanity+1;
  } else {
    std::cout << "Invalid End Point!!\n";
  }
  return(sanity == 2);
}

void Map::setx(int value) {
  x = value;
}
int Map::getx() {
  return x;
}

void Map::sety(int value) {
  y = value;
}

int Map::gety() {
  return y;
}
