/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <graph_core/datastructure/vector.h>

namespace pathplan
{

Vector::Vector(bool mode):
  NearestNeighbors(mode)
{

}

void Vector::insert(const NodePtr& node)
{
  nodes_.push_back(node);
  size_++;
  return;
}


void Vector::nearestNeighbor(const Eigen::VectorXd& configuration,
                             NodePtr &best,
                             double &best_distance)
{
  best_distance=std::numeric_limits<double>::infinity();
  for (const NodePtr& n: nodes_)
  {
    double dist=l2_dist(n,configuration);
    //if time_avoid, need to check avoid ints between nodes
    if (dist<best_distance)
    {      
      best=n;
      best_distance=dist;
    }
  }
}

//get nearest nodes based on the cost function
//this used to simply be the l2 cost function (shortest path)
//JF - I added the option to specify a cost function so I could choose to find neighbors nearest in time instead of l2 distance
//JF - Nearest in time considers slow downs to avoid predicted obstacles, so it may not be linearly related to l2 distance
std::multimap<double, pathplan::NodePtr> Vector::near(const Eigen::VectorXd& configuration,
                                  const double& radius)
{
  // PATH_COMMENT_STREAM("near radius "<<radius);
  std::multimap<double, pathplan::NodePtr> nodes;
  for (const NodePtr& n: nodes_)
  {
    // ROS_INFO_STREAM("node ptr "<<n);
    double dist=l2_dist(n,configuration);  //get cost between node n and config, either l2 distance or time depending on cost_fn
    if (dist<radius) //if cost is close enough, then node is viable parent
    {
      nodes.insert(std::pair<double, pathplan::NodePtr>(dist,n)); //build table of nearest nodes and cost for edge: nearest node -> node n
    }
  }
  return nodes;
}

std::multimap<double, NodePtr> Vector::kNearestNeighbors(const Eigen::VectorXd& configuration,
                                        const size_t& k)
{
  std::multimap<double, NodePtr> nodes;
  for (const NodePtr& n: nodes_)
  {
    double dist=(n->getConfiguration()-configuration).norm();
    nodes.insert(std::pair<double, pathplan::NodePtr>(dist,n));
  }
  if (nodes.size()<k)
    return nodes;

  std::multimap<double, NodePtr> m2(nodes.begin(), std::next(nodes.begin(), k));
  return m2;
}


bool Vector::findNode(const NodePtr& node)
{
  return  (std::find(nodes_.begin(),nodes_.end(),node)!=nodes_.end());
}


bool Vector::deleteNode(const NodePtr& node,
                        const bool& disconnect_node)
{
  std::vector<NodePtr>::iterator it;
  it=std::find(nodes_.begin(),nodes_.end(),node);
  if (it==nodes_.end())
    return false;

  size_--;
  delete_nodes_++;

  nodes_.erase(it);
  return true;
}

bool Vector::restoreNode(const NodePtr& node)
{
  return false;
}

std::vector<NodePtr> Vector::getNodes()
{
  return nodes_;
}

void Vector::disconnectNodes(const std::vector<NodePtr>& white_list)
{
  for (NodePtr& n: nodes_)
  {
    if (std::find(white_list.begin(),white_list.end(),n)==white_list.end())
      n->disconnect();
  }
}

}  // namespace pathplan