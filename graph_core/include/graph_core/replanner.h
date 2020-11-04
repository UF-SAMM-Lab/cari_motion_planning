#pragma once
#ifndef REPLANNER_H
#define REPLANNER_H
#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <graph_core/util.h>
#include <graph_core/graph/tree.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/connection.h>
#include <graph_core/graph/node.h>
#include <graph_core/metrics.h>
#include <graph_core/solvers/tree_solver.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/collision_checker.h>

namespace pathplan
{
    class Replanner;
    typedef std::shared_ptr<Replanner> ReplannerPtr;

    class Replanner: public std::enable_shared_from_this<Replanner>
    {
        protected:
            Eigen::VectorXd current_configuration_;        // current robot conf
            PathPtr current_path_;                         // current path traveled by the robot
            PathPtr replanned_path_;                        // replanned path
            std::vector<PathPtr> replanned_paths_vector_;   // vector of the 10 best replanned paths
            std::vector<PathPtr> other_paths_;             // initial available paths
            std::vector<PathPtr> admissible_other_paths_;  // available paths
            std::vector<NodePtr> examined_nodes_;          // node considered during the replanning
            std::vector<NodePtr> nodes_set_;               // set of available nodes
            TreeSolverPtr solver_;                         // solver
            MetricsPtr metrics_;
            CollisionCheckerPtr checker_;
            Eigen::VectorXd lb_;
            Eigen::VectorXd ub_;


            //time_first_sol
            //time_replanning


        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Replanner();

            PathPtr getReplannedPath()
            {
                return replanned_path_;
            }  
            void setCurrentConf(Eigen::VectorXd& q)
            {
                current_configuration_ = q;
            }

            ReplannerPtr pointer()
            {
                return shared_from_this();
            }

            bool pathSwitch(const NodePtr& node, const bool& succ_node, PathPtr& new_path, PathPtr& subpath_from_path2, int& connected2path_number);
            bool informedOnlineReplanning(const int& informed, const bool& succ_node);

    };
}

#endif // REPLANNER_H
