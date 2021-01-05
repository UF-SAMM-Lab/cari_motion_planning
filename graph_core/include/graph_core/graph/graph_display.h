/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
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

#pragma once
#include <graph_core/graph/tree.h>
#include <graph_core/graph/path.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/planning_scene/planning_scene.h>
#include <eigen_conversions/eigen_msg.h>

namespace pathplan
{
#define DISPLAYTIME 0.1
class Display;
typedef std::shared_ptr<Display> DisplayPtr;
class Display: public std::enable_shared_from_this<Display>
{
protected:
  planning_scene::PlanningScenePtr planning_scene_;
  std::string group_name_;
  std::string last_link_;
  int marker_id_;
  std::vector<double> node_marker_scale_;
  std::vector<double> connection_marker_scale_;
  std::vector<double> tree_marker_scale_;
  ros::Publisher marker_pub_;
  ros::NodeHandle nh_;
  moveit::core::RobotStatePtr state_;

  void displayTreeNode(const NodePtr& n,
                       const Direction& direction,
                       std::vector<geometry_msgs::Point> &points
                       );
public:
  Display(const planning_scene::PlanningScenePtr planning_scene,
          const std::string& group_name,
          const std::string& last_link);

  void clearMarkers(const std::string &ns="pathplan");
  void clearMarker(const int& id,const std::string& ns="pathplan");
  int displayNode(const NodePtr& n,
                   const std::string& ns="pathplan",
                   const std::vector<double>& marker_color = {1,0,0,1.0},
                   const bool& plot_state=false);
  int displayConnection(const ConnectionPtr& conn,
                         const std::string& ns="pathplan",
                         const std::vector<double>& marker_color= {1,0,0,1.0},
                         const bool& plot_state=false);
  int displayPath(const PathPtr& path,
                   const std::string& ns="pathplan",
                   const std::vector<double>& marker_color= {1,0,0,1.0},
                   const bool& plot_state=false);

  int displayTree(const TreePtr& tree,
                   const std::string& ns="pathplan",
                   const std::vector<double>& marker_color= {1,0,0,1.0});
};
}



