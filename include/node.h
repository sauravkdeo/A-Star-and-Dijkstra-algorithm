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
#ifndef _HOME_SAURAV_WORKSPACE_SRC_PROJ2_SAURAV_KUMAR_CPP_INCLUDE_NODE_H_
#define _HOME_SAURAV_WORKSPACE_SRC_PROJ2_SAURAV_KUMAR_CPP_INCLUDE_NODE_H_
#include <../include/map.h>
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


// Node struct with node matrix , level and cost
class Node {
 public:
  // stores parent node of current node
  // helps in tracing path when the goal is achieved
  Coord position;

  // stores the number of moves so far
  double costtocome;
  double costtogo;
  struct comp {
    bool operator()(const Node* lhs, const Node* rhs) const {
      return (lhs->costtocome + lhs->costtogo) >
      (rhs->costtocome + rhs->costtogo);
    }
  };

  Node* parent;

  Node* childup;
  Node* childdown;
  Node* childright;
  Node* childleft;
  Node* childupleft;
  Node* childupright;
  Node* childdownright;
  Node* childdownleft;
  //  static priority_queue<Node*, std::vector<Node*>, comp1> pq;

  Map * gridmap;

  double calculateastarcosttogo(Node*, Coord);
  Node* newnode(Coord, Node*, double,
                std::map<int, std::map<int, Node*> >*, Map*,
                priority_queue<Node*, std::vector<Node*>, Node::comp>*);
  void traceback(Node*);

  bool ispositionsame(int, int, Coord);
  bool moveup(Node*);
  bool moveupright(Node*);
  bool moveright(Node*);
  bool movedownright(Node*);
  bool movedown(Node*);

  bool movedownleft(Node*);
  bool moveleft(Node*);
  bool moveupleft(Node*);

  bool nearvicinity(Node*);
  void explorechild(Node*,  priority_queue<Node*,
                    std::vector<Node*>, comp>*, std::map<int,
                    std::map<int, Node*> >*, Map*);

  void solve(Coord position, Map* mp);
};
#endif  // _HOME_SAURAV_WORKSPACE_SRC_PROJ2_SAURAV_KUMAR_CPP_INCLUDE_NODE_H_
