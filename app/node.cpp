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



double Node::calculateastarcosttogo(Node* node, Coord goal) {
  Coord coordinate = node->position;
  return (sqrt(pow((coordinate.get_x()-goal.get_x()), 2) +
               pow((coordinate.get_y()-goal.get_y()), 2)));
}


Node* Node::newnode(Coord coordinate, Node* parent, double stepCost,
              std::map<int, std::map<int, Node*> >* ngridmap, Map* arraymap,
              priority_queue<Node*, std::vector<Node*>, comp>* pq) {
  if ((*ngridmap)[coordinate.get_x()][coordinate.get_y()]) {
    if (((*ngridmap)[coordinate.get_x()][coordinate.get_y()]->costtocome) >
    (parent->costtocome+stepCost)) {
      (*ngridmap)[coordinate.get_x()][coordinate.get_y()]->parent = parent;
      (*ngridmap)[coordinate.get_x()][coordinate.get_y()]->costtocome =
          parent->costtocome+stepCost;
    }
  }
  if (!(*ngridmap)[coordinate.get_x()][coordinate.get_y()]) {
    Node* node = new Node;
    node->position = coordinate;
    node->gridmap = arraymap;
    node->parent = parent;

    if (parent == NULL) {
      node->costtocome = stepCost;
    }
    if (parent != NULL) {
      node->costtocome = parent->costtocome+stepCost;
    }

    if (node->gridmap->getoption() == 0) {
      node->costtogo = 0;
    }
    if (node->gridmap->getoption() == 1) {
      node->costtogo =
          sqrt
          (pow((coordinate.get_x()-node->gridmap->getendcoord().get_x()), 2) +
           pow((coordinate.get_y()-node->gridmap->getendcoord().get_y()), 2));
    }
    node->gridmap->fillmap(coordinate, 3);
    (*pq).push(node);

    (*ngridmap)[coordinate.get_x()][coordinate.get_y()] = node;
  }
  return (*ngridmap)[coordinate.get_x()][coordinate.get_y()];
}



// Comparison object to be used to order the heap

// Function to solve N*N - 1 puzzle algorithm using
// Branch and Bound. x and y are blank tile coordinates
// in initial state
void Node::traceback(Node* trace) {
  trace->gridmap->updatemap(trace->position, trace->parent->position, 5);
  if (trace->parent->position.get_x() !=
      trace->gridmap->getstartingCoord().get_x() ||
      trace->parent->position.get_y() !=
          trace->gridmap->getstartingCoord().get_y()) {
    traceback(trace->parent);
  }
}

bool Node::ispositionsame(int startx, int starty, Coord end) {
  return(startx == end.get_x() && starty == end.get_y());
}

bool Node::moveup(Node* node) {
  int count = 0;
  for (int i = 1; i <= node->gridmap->getresolution(); i++) {
    if (node->position.get_y()+i < node->gridmap->getgridmapheight()) {
      if (node->gridmap->getgridmap
          (node->position.get_x(), node->position.get_y()+i) > 0) {
        count = count+1;
      }
    }
  }
  if (count == node->gridmap->getresolution()) {
    return true;
  } else {
    return false;
  }
}
bool Node::moveupright(Node* node) {
  int count = 0;
  for (int i = 1; i <= node->gridmap->getresolution(); i++) {
    if (node->position.get_y()+i < node->gridmap->getgridmapheight() &&
        node->position.get_x()+i < node->gridmap->getgridmapwidth()) {
      if (node->gridmap->getgridmap(node->position.get_x()+i,
                                    node->position.get_y()+i) > 0) {
        count = count+1;
      }
    }
  }
  if (count == node->gridmap->getresolution()) {
    return true;
  } else {
    return false;
  }
}

bool Node::moveright(Node* node) {
  int count = 0;
  for (int i = 1; i <= node->gridmap->getresolution(); i++) {
    if (node->position.get_x()+i < node->gridmap->getgridmapwidth()) {
      if (node->gridmap->getgridmap
          (node->position.get_x()+i, node->position.get_y()) > 0) {
        count = count+1;
      }
    }
  }
  if (count == node->gridmap->getresolution()) {
    return true;
  } else {
    return false;
  }
}

bool Node::movedownright(Node* node) {
  int count = 0;
  for (int i = 1; i <= node->gridmap->getresolution(); i++) {
    if (node->position.get_x()+i < node->gridmap->getgridmapwidth() &&
        node->position.get_y()-i >= 0 ) {
      if (node->gridmap->getgridmap
          (node->position.get_x()+i, node->position.get_y()-i) > 0) {
        count = count+1;
      }
    }
  }
  if (count == node->gridmap->getresolution()) {
    return true;
  } else {
    return false;
  }
}

bool Node::movedown(Node* node) {
  int count = 0;
  for (int i = 1; i <= node->gridmap->getresolution(); i++) {
    if (node->position.get_y()-i >= 0) {
      if (node->gridmap->getgridmap
          (node->position.get_x(), node->position.get_y()-i) > 0) {
        count = count+1;
      }
    }
  }
  if (count == node->gridmap->getresolution()) {
    return true;
  } else {
    return false;
  }
}
bool Node::movedownleft(Node* node) {
  int count = 0;
  for (int i = 1; i <= node->gridmap->getresolution(); i++) {
    if (node->position.get_y()-i >= 0 && node->position.get_x()-i >= 0) {
      if (node->gridmap->getgridmap
          (node->position.get_x()-i, node->position.get_y()-i) > 0) {
        count = count+1;
      }
    }
  }
  if (count == node->gridmap->getresolution()) {
    return true;
  } else {
    return false;
  }
}

bool Node::moveleft(Node* node) {
  int count = 0;
  for (int i = 1; i <= node->gridmap->getresolution(); i++) {
    if (node->position.get_x()-i >= 0) {
      if (node->gridmap->getgridmap(node->position.get_x()-i,
                                    node->position.get_y()) > 0) {
        count = count+1;
      }
    }
  }
  if (count == node->gridmap->getresolution()) {
    return true;
  } else {
    return false;
  }
}
bool Node::moveupleft(Node* node) {
  int count = 0;
  for (int i = 1; i <= node->gridmap->getresolution(); i++) {
    if (node->position.get_x()-i >= 0 &&
        node->position.get_y()+i < node->gridmap->getgridmapheight() ) {
      if (node->gridmap->getgridmap
          (node->position.get_x()-i, node->position.get_y()+i) > 0) {
        count = count+1;
      }
    }
  }
  if (count == node->gridmap->getresolution()) {
    return true;
  } else {
    return false;
  }
}

bool Node::nearvicinity(Node* node) {
  int count = 0;
  if (abs(node->gridmap->getendcoord().get_x()-
          node->position.get_x()) < node->gridmap->getresolution()
      && abs(node->gridmap->getendcoord().get_y()-
             node->position.get_y()) < node->gridmap->getresolution() ) {
    double deltax =
        (node->gridmap->getendcoord().get_x()-node->position.get_x());
    double deltay =
        (node->gridmap->getendcoord().get_y()-node->position.get_y());
    double maximum = std::max(abs(deltax), abs(deltay));
    deltax = deltax/maximum;
    deltay = deltay/maximum;
    for (int i = 0; i < maximum; i++) {
      if (node->gridmap->getgridmap(
          (node->position.get_x()+ceil(i*deltax)),
          (node->position.get_y()+ceil(i*deltay))) > 0) {
        count = count +1;
      }
    }

    if (count == maximum) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

void Node::explorechild(Node* root,  priority_queue<Node*,
                  std::vector<Node*>, comp>* pq, std::map<int,
                  std::map<int, Node*> >* ngmap, Map* araymap) {
  double stepcost;
  Coord newlocation;
  if (nearvicinity(root)) {
    stepcost = sqrt(pow((root->gridmap->getendcoord().get_y()-
        root->position.get_y()), 2)+
                    pow((root->gridmap->getendcoord().get_x()-
                        root->position.get_x()), 2));
    newlocation.set_x(root->gridmap->getendcoord().get_x());
    newlocation.set_y(root->gridmap->getendcoord().get_y());

    newnode(newlocation, root, stepcost, ngmap, araymap, pq);
  }

  if (moveup(root)) {
    stepcost = 1*root->gridmap->getresolution();
    newlocation.set_x(root->position.get_x());
    newlocation.set_y(root->position.get_y()+root->gridmap->getresolution());
    root->childup = newnode(newlocation, root, stepcost, ngmap, araymap, pq);
  } else {
    root->childup = NULL;
  }
  if (moveupright(root)) {
    stepcost = M_SQRT2*root->gridmap->getresolution();
    newlocation.set_x(root->position.get_x()+root->gridmap->getresolution());
    newlocation.set_y(root->position.get_y()+root->gridmap->getresolution());
    root->childupright =
        newnode(newlocation, root, stepcost, ngmap, araymap, pq);
  } else {
    root->childupright = NULL;
  }
  if (moveright(root)) {
    stepcost = 1*root->gridmap->getresolution();
    newlocation.set_x(root->position.get_x()+root->gridmap->getresolution());
    newlocation.set_y(root->position.get_y());
    root->childright = newnode(newlocation, root, stepcost, ngmap, araymap, pq);

  } else {
    root->childright = NULL;
  }
  if (movedownright(root)) {
    stepcost = M_SQRT2*root->gridmap->getresolution();
    newlocation.set_x(root->position.get_x()+root->gridmap->getresolution());
    newlocation.set_y(root->position.get_y()-root->gridmap->getresolution());
    root->childdownright =
        newnode(newlocation, root, stepcost, ngmap, araymap, pq);
  } else {
    root->childdownright = NULL;
  }
  if (movedown(root)) {
    stepcost = 1*root->gridmap->getresolution();
    newlocation.set_x(root->position.get_x());
    newlocation.set_y(root->position.get_y()-root->gridmap->getresolution());
    root->childdown = newnode(newlocation, root, stepcost, ngmap, araymap, pq);
  } else {
    root->childdown = NULL;
  }
  if (movedownleft(root)) {
    stepcost = M_SQRT2*root->gridmap->getresolution();
    newlocation.set_x(root->position.get_x()-root->gridmap->getresolution());
    newlocation.set_y(root->position.get_y()-root->gridmap->getresolution());
    root->childdownleft =
        newnode(newlocation, root, stepcost, ngmap, araymap, pq);
  } else {
    root->childdownleft = NULL;
  }
  if (moveleft(root)) {
    stepcost = 1*root->gridmap->getresolution();
    newlocation.set_x(root->position.get_x()-root->gridmap->getresolution());
    newlocation.set_y(root->position.get_y());
    root->childleft = newnode(newlocation, root, stepcost, ngmap, araymap, pq);
  } else {
    root->childleft = NULL;
  }
  if (moveupleft(root)) {
    stepcost = M_SQRT2*root->gridmap->getresolution();
    newlocation.set_x(root->position.get_x()-root->gridmap->getresolution());
    newlocation.set_y(root->position.get_y()+root->gridmap->getresolution());
    root->childupleft =
        newnode(newlocation, root, stepcost, ngmap, araymap, pq);
  } else {
    root->childupleft = NULL;
  }
}

void Node::solve(Coord position, Map* mp) {
  std::map<int, std::map<int, Node*> > ngm;
  priority_queue<Node*, std::vector<Node*>, comp> pq;
  // create a root node and calculate its cost
  newnode(position, NULL, 0.0, &ngm, mp, &pq);


  while (!pq.empty()) {
    // Find a live node with least estimated cost
    Node* min = pq.top();

    // The found node is deleted from the list of
    // live nodes
    pq.pop();

    // if min is an answer node
    if (min->position.get_x()== min->gridmap->getendcoord().get_x() &&
        min->position.get_y()== min->gridmap->getendcoord().get_y()) {
      traceback(min);
      return;
    }
    explorechild(min, &pq, &ngm, mp);
  }
  if (pq.empty()) {
    cout << "No Solution !!";
  }
}
