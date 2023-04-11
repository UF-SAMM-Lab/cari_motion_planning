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

#include <graph_core/graph/tree.h>

namespace pathplan
{

  Tree::Tree(const NodePtr &root,
             const double &max_distance,
             const CollisionCheckerPtr &checker,
             const MetricsPtr &metrics,
             const bool &use_kdtree,
             const bool &use_time_cost,
             const bool &use_net) : root_(root),
                                          use_kdtree_(use_kdtree),
                                          max_distance_(max_distance),
                                          checker_(checker),
                                          metrics_(metrics),
                                          time_avoid_(use_time_cost),
                                          use_net(use_net)
  {
    if (use_kdtree)
    {
      nodes_ = std::make_shared<KdTree>(use_time_cost);
    }
    else
    {
      nodes_ = std::make_shared<Vector>(use_time_cost);
    }
    PATH_COMMENT_STREAM("root min time:" << root->min_time);
    nodes_->insert(root);
    double dimension = root->getConfiguration().size();
    k_rrt_ = 1.1 * std::pow(2.0, dimension + 1) * std::exp(1) * (1.0 + 1.0 / dimension);
  }

  NodePtr Tree::findClosestNode(const Eigen::VectorXd &configuration)
  {
    if (!time_avoid_)
    {
      return nodes_->nearestNeighbor(configuration);
    }
    else
    {
      double best_distance = std::numeric_limits<double>::infinity();
      NodePtr best;
      std::vector<NodePtr> all_nodes = getNodes();
      //JF-rewrite to check all n->all_nodes connections in one call to pc_avoid_checker->checkMultiplePaths()
      //return Eigen::VectorXd last_pass_time for all connections
      //can get dist with matrix algebra for all connections
      for (const NodePtr &n : all_nodes)
      {
        if (n == goal_node_)
          continue;
        // is it connecting to an a node with inf cost?
        double dist = nodes_->l2_dist(n, configuration);
        // if time_avoid, need to check avoid ints between nodes
        if (dist < best_distance)
        {
          std::vector<Eigen::Vector3f> avoid_ints;
          float last_pass_time;
          double min_time_to_node = nodes_->time_dist(n, configuration);
          Eigen::VectorXd parent_config = n->getConfiguration();
          metrics_->pc_avoid_checker->checkPath(parent_config, configuration, avoid_ints, last_pass_time);
          bool connection_possible = min_time_to_node < last_pass_time;
          // ROS_INFO_STREAM("connection possible:"<<connection_possible<<", min time to node:"<<min_time_to_node<<", last pass time:"<<last_pass_time);
          if (connection_possible)
          {
            best = n;
            best_distance = dist;
          }
        }
      }

      //look up where min_time_to_node < last_pass_time and get id of min dist from remaining list
      return best;
    }
  }

  // this function finds the node nearest configuration and then
  // takes a step in the direction of the vector from config->node
  // if collision occurs, returns false
  bool Tree::tryExtend(const Eigen::VectorXd &configuration,
                       Eigen::VectorXd &next_configuration,
                       NodePtr &closest_node)
  {
    // find nearest neighbor from which to take a random step
    // for time-avoidance, this returns the nearest node in time,
    // otherwise, nearest in l2 distance
    closest_node = findClosestNode(configuration);
    // std::cout<<"try extend closest node cfg:\n"<<closest_node->getConfiguration()<<std::endl;
    if (!closest_node)
    {
      std::cout << "no closest node to:" << configuration.transpose() << std::endl;
      return false;
    }
    return tryExtendFromNode(configuration, next_configuration, closest_node);
  }

  bool Tree::tryExtendFromNode(const Eigen::VectorXd &configuration,
                               Eigen::VectorXd &next_configuration,
                               NodePtr &node)
  {
    assert(node);
    double distance = selectNextConfiguration(configuration, next_configuration, node);

    // std::cout<<"next_cfg: "<<next_configuration.transpose()<<std::endl;
    // PATH_COMMENT_STREAM("try ext dist "<< distance);
    if (distance < tolerance_)
    { // if difference in config is small enough that it is imposssible for an obstacle between configs
      return true;
      // PATH_COMMENT_STREAM("too close");
    }
    else
    {
      // check path between node and next_config for intermediate collisions
      if (checker_->checkPath(node->getConfiguration(), next_configuration))
      {
        return true;
        // PATH_COMMENT_STREAM("no collision between nodes");
      }
      // std::cout<<"collision_for_cfg: "<<configuration.transpose()<<std::endl;
      // PATH_COMMENT_STREAM("collision between nodes");
    }
    return false;
  }

  double Tree::selectNextConfiguration(const Eigen::VectorXd &configuration,
                                       Eigen::VectorXd &next_configuration,
                                       const NodePtr &node)
  {
    assert(node);
    // must return l2 distance since it is used to determine if intermiediate configs need collision checked
    double distance = (node->getConfiguration() - configuration).norm();

    // if nodes are close enough together then the new configuratin is the old configuration? seems odd
    if (distance < tolerance_)
    {
      next_configuration = configuration;
    }
    // again, if nodes are close enough together then the new config is the old config? also seems odd
    else if (distance < max_distance_)
    {
      next_configuration = configuration;
    }
    else
    { // take step of random length? looks like all step lengths are max_distance_ thought
      next_configuration = node->getConfiguration() + (configuration - node->getConfiguration()) / distance * max_distance_;
    }

    return distance;
  }
  // this function makes a new edge between a new random node and its nearest parent node
  bool Tree::extendOnly(NodePtr &closest_node, NodePtr &new_node, ConnectionPtr &connection)
  {
    ConnectionPtr conn; // empty connection
    // get edge cost, for time-avoid this is the min time to reach the new node, considering avoidance intervals
    double cost;
    double n_time = 0;
    if (time_avoid_)
    {
      std::vector<Eigen::Vector3f> avoid_ints;
      float min_human_dist;
      float last_pass_time;
      // is it connecting to an a node with inf cost?
      cost = metrics_->cost(closest_node, new_node, n_time, avoid_ints, last_pass_time, min_human_dist);
      // PATH_COMMENT_STREAM("new node cost:"<<cost);
      if (cost == std::numeric_limits<double>::infinity())
        return false;
      // new_node->min_time = std::min(closest_node->min_time+cost,new_node->min_time);
      conn = std::make_shared<Connection>(closest_node, new_node);
      conn->setMinTime(inv_max_speed_, min_accel_time);
      conn->setAvoidIntervals(avoid_ints, last_pass_time, min_human_dist);
      // PATH_COMMENT_STREAM("adding connection:"<<cost<<", "<<n_time<<","<<avoid_ints.size());
      // PATH_COMMENT_STREAM("adding connection:"<<cost<<", "<<n_time<<","<<avoid_ints.size());
      // for (int i=0;i<avoid_ints.size();i++) {
      //   std::cout<<avoid_ints[i].transpose()<<std::endl;
      // }
      // PATH_COMMENT_STREAM("end of ints");
      conn->setParentTime(n_time);
      conn->add();
      conn->setCost(cost);
    }
    else
    {
      cost = metrics_->cost(closest_node, new_node);
      conn = std::make_shared<Connection>(closest_node, new_node);
      // PATH_COMMENT_STREAM("adding connection:" << closest_node << "," << new_node);
      conn->add();
      conn->setCost(cost);
    }

    connection = conn;
    addNode(new_node);
    // ROS_INFO_STREAM("added node1:"<<new_node);
    // addNode(new_node,false);

    return true;
  }

  void Tree::directSolution(NodePtr &goal_node_)
  {
    ConnectionPtr conn;
    extendOnly(root_, goal_node_, conn);
  }

  bool Tree::extend(const Eigen::VectorXd &configuration, NodePtr &new_node)
  {
    ConnectionPtr conn;
    return extend(configuration, new_node, conn);
  }

  bool Tree::extend(const Eigen::VectorXd &configuration, NodePtr &new_node, ConnectionPtr &connection)
  {
    NodePtr closest_node;
    Eigen::VectorXd next_configuration;
    // attempt connection between config and its closest node
    // if ok, next_config will be a step of random len in direction nearest node->config
    if (!tryExtend(configuration,
                   next_configuration,
                   closest_node))
    {
      connection = NULL;
      return false;
    }
    // std::cout<<"next_cfg: "<<next_configuration.transpose()<<std::endl;
    // if extension passed, create new node at the next config
    new_node = std::make_shared<Node>(next_configuration);
    // ROS_INFO("extending only...................................");
    return extendOnly(closest_node, new_node, connection);
  }

  bool Tree::extendToNode(const NodePtr &node,
                          NodePtr &new_node)
  {
    NodePtr closest_node;
    Eigen::VectorXd next_configuration;
    // this appears to be the steering function
    //
    //  PATH_COMMENT_STREAM("node cfg:"<<node->getConfiguration());

    if (!tryExtend(node->getConfiguration(),
                   next_configuration,
                   closest_node))
    {
      return false;
    }
    // PATH_COMMENT_STREAM("next cfg:"<<next_configuration);
    // PATH_COMMENT_STREAM("node "<<*node);
    // PATH_COMMENT_STREAM("closest node "<<*closest_node);
    new_node = std::make_shared<Node>(next_configuration);
    // addNode(new_node);//addNode(new_node,false);
    // ROS_INFO_STREAM("added node:"<<new_node<<"\n"<<*new_node);

    bool attached = false;
    if ((next_configuration - node->getConfiguration()).norm() < tolerance_)
    {
      attached = true;
      new_node = node;
      // std::cout<<"attaching:"<<*new_node;
    }
    else
    {
      new_node = std::make_shared<Node>(next_configuration);
      addNode(new_node); // addNode(new_node,false);
    }

    double cost;
    double n_time = 0;
    std::vector<Eigen::Vector3f> avoid_ints;
    float min_human_dist;
    float last_pass_time;
    if (time_avoid_)
    {
      cost = metrics_->cost(closest_node, new_node, n_time, avoid_ints, last_pass_time, min_human_dist);
    }
    else
    {
      cost = metrics_->cost(closest_node, new_node);
    }
    // PATH_COMMENT_STREAM("parent->child extension cost:"<<cost);
    ConnectionPtr conn = std::make_shared<Connection>(closest_node, new_node);
    // PATH_COMMENT_STREAM("adding connection2:"<<cost<<", "<<n_time<<","<<avoid_ints.size());
    // for (int i=0;i<avoid_ints.size();i++) {
    //   std::cout<<avoid_ints[i].transpose()<<std::endl;
    // }
    // PATH_COMMENT_STREAM("end of ints");
    if (!time_avoid_) {
      if (!new_node->parent_connections_.empty()) new_node->parent_connections_[0]->remove();
    }
    conn->add();
    if (time_avoid_)
      conn->setParentTime(n_time);
    // PATH_COMMENT_STREAM("setting cost for connection");
    conn->setCost(cost);
    if (time_avoid_)
    {
      conn->setAvoidIntervals(avoid_ints, last_pass_time, min_human_dist);
      conn->setMinTime(inv_max_speed_, min_accel_time);
    }
    // PATH_COMMENT_STREAM("new node connection done");

    return true;
  }

  bool Tree::connect(const Eigen::VectorXd &configuration, NodePtr &new_node)
  {
    bool success = true;
    while (success)
    {
      NodePtr tmp_node;
      success = extend(configuration, tmp_node);
      if (success)
      {
        new_node = tmp_node;
        if ((new_node->getConfiguration() - configuration).norm() < tolerance_)
          return true;
      }
    }
    return false;
  }

  bool Tree::informedExtend(const Eigen::VectorXd &configuration, NodePtr &new_node, const Eigen::VectorXd &goal, const double &cost2beat, const double &bias)
  {
    struct extension
    {
      NodePtr tree_node;
      Eigen::VectorXd new_conf;
      double distance;
    };

    std::multimap<double, NodePtr> closest_nodes_map = nearK(configuration);
    assert(closest_nodes_map.size() > 0);

    double heuristic, distance, cost2node;
    Eigen::VectorXd new_configuration;
    std::multimap<double, extension> best_nodes_map;

    for (const std::pair<double, NodePtr> &n : closest_nodes_map)
    {
      distance = selectNextConfiguration(configuration, new_configuration, n.second);
      cost2node = costToNode(n.second);
      heuristic = bias * distance + (1 - bias) * cost2node;

      extension ext;
      ext.tree_node = n.second;
      ext.new_conf = new_configuration;
      ext.distance = distance;

      if ((cost2node + distance + (goal - new_configuration).norm()) < cost2beat) // if and only if the new conf underestimation of cost is less than the cost to beat it is added to the tree
        best_nodes_map.insert(std::pair<double, extension>(heuristic, ext));      // give priority to parents with the best heuristics
    }

    bool extend_ok = false;
    extension ext;
    for (const std::pair<double, extension> &n : best_nodes_map)
    {
      ext = n.second;
      if (ext.distance < tolerance_)
      {
        extend_ok = true;
        break;
      }
      else
      {
        if (checker_->checkPath(ext.tree_node->getConfiguration(), ext.new_conf))
        {
          extend_ok = true;
          break;
        }
      }
    }

    if (extend_ok)
    {
      ConnectionPtr connection;
      new_node = std::make_shared<Node>(ext.new_conf);
      return extendOnly(ext.tree_node, new_node, connection);
    }
    else
      return false;
  }

  bool Tree::connectToNode(const NodePtr &node, NodePtr &new_node, const double &max_time)
  {
    ros::WallTime tic = ros::WallTime::now();

    if (max_time <= 0.0)
    {
      // PATH_COMMENT_STREAM("max time was <= 0.0");
      return false;
    }

    bool success = true;
    while (success)
    {
      NodePtr tmp_node;
      // PATH_COMMENT_STREAM("calling extend");
      // std::cout<<*node;
      success = extendToNode(node, tmp_node); // tmp_node will be a new node in direction of parent->node
      // std::cout<<"after extend:"<<*node;
      if (success)
      {
        // PATH_COMMENT_STREAM("extended\n"<<*node<<"\n to \n"<<*tmp_node);
        new_node = tmp_node;

        if ((new_node->getConfiguration() - node->getConfiguration()).norm() < tolerance_)
        {
          return true;
        }
      }
      // PATH_COMMENT_STREAM("max time:"<<max_time);
      if ((ros::WallTime::now() - tic).toSec() >= 0.98 * max_time)
        break;
    }
    return false;
  }

  bool Tree::checkPathToNode(const NodePtr &node, std::vector<ConnectionPtr> &checked_connections)
  {
    std::vector<ConnectionPtr> connections_to_check;
    std::vector<ConnectionPtr> path_connections = getConnectionToNode(node);

    for (const ConnectionPtr conn : path_connections)
    {
      bool already_checked = false;
      for (const ConnectionPtr checked_conn : checked_connections) // check if the connection considered has not already been verified
      {
        if (checked_conn == conn)
        {
          already_checked = true;
          if (checked_conn->getCost() == std::numeric_limits<double>::infinity())
            return false;

          break;
        }
      }

      if (!already_checked)
        connections_to_check.push_back(conn);
    }

    for (const ConnectionPtr conn : connections_to_check)
    {
      if (!checker_->checkConnection(conn))
      {
        conn->setCost(std::numeric_limits<double>::infinity());
        checked_connections.push_back(conn);

        return false;
      }
      else
      {
        conn->setCost(metrics_->cost(conn->getParent(), conn->getChild()));
        checked_connections.push_back(conn);
      }
    }

    return true;
  }

  // this function has 2 purposes:
  // 1) for a new node which has no parent, find the best parent node in terms of minimum cost for parent->new node
  // 2) for existing nodes, look for better parent nodes within a radius
  bool Tree::rewireOnly(NodePtr &node, double r_rewire, const int &what_rewire)
  {
    // std::vector<NodePtr> all_nodes = getNodes();
    // std::cout<<"all nodes:\n";
    // for (int i=0;i<all_nodes.size();i++) {
    //   std::cout<<all_nodes[i]<<std::endl;
    //   std::cout<<*all_nodes[i]<<std::endl;
    // }
    if (what_rewire > 2 || what_rewire < 0)
    {
      ROS_ERROR("what_rewire parameter should be 0,1 or 2");
      assert(0);
      return false;
    }

    bool rewire_parent;
    bool rewire_children;

    switch (what_rewire)
    {
    case 0:
      rewire_parent = true;
      rewire_children = true;
      break;
    case 1:
      rewire_parent = true;
      rewire_children = false;
      break;
    case 2:
      rewire_parent = false;
      rewire_children = true;
      break;
    }

    if (node == root_)
      rewire_parent = false; // start node has no parents
    // get nodes in neighborhood of node based on cost fn, either time based or l2 distance
    std::multimap<double, NodePtr> near_nodes = near(node, r_rewire);
    // int num_nodes = nodes_->size();
    // // ROS_INFO_STREAM("num nodes:"<<num_nodes);
    // std::multimap<double, NodePtr> near_nodes = near(node, r_rewire/sqrt(num_nodes));
    
    // JF - I assume that for new nodes, cost to node is inf
    //  ROS_INFO_STREAM("node cost"<<*node);
    double cost_to_node = costToNode(node);
    bool improved = false;

    double cost_node_to_near;
    if (rewire_parent)
    {
      // ROS_INFO_STREAM("node parents:"<<node->getParents().size());
      // ROS_INFO_STREAM("try to find a better parent between "<<near_nodes.size()<<" nodes w/ time avoid:"<<time_avoid_);
      NodePtr nearest_node = node->getParents().at(0);
      // std::cout<<"node:"<<*node<<std::endl;
      // std::cout<<"nearest node:"<<*nearest_node<<std::endl;
      // loop over all nearest nodes
      
      // JF - determine all avoidance intervals between nodes before loop
      if (time_avoid_ && use_net) {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        std::vector<std::tuple<const NodePtr,const NodePtr,double,std::vector<Eigen::Vector3f>,float,float,double>> connection_datas;
        connection_datas.reserve(near_nodes.size());
        for (const std::pair<double, NodePtr> &p : near_nodes) {
          const NodePtr &n = p.second;
          if (n == goal_node_)
            continue;
          // check to prevent a node from becoming its own parent
          if (n == nearest_node)
            continue;
          if (n == node)
            continue;          
          if (!checker_->checkPath(n->getConfiguration(), node->getConfiguration()))
            continue;

          connection_datas.emplace_back(n,node,0,std::vector<Eigen::Vector3f>(),0,0,0);
        }
        metrics_->cost(connection_datas);
        // std::cout<<"connections checked:"<<connection_datas.size()<<std::endl;

        for (int c=0;c<connection_datas.size();c++)
        {
          std::tuple<const NodePtr,const NodePtr,double,std::vector<Eigen::Vector3f>,float,float,double> conn_data = connection_datas[c];
          const NodePtr &n = std::get<0>(conn_data); // get near node from pair
          // if (n == goal_node_)
          //   continue;
          // // check to prevent a node from becoming its own parent
          // if (n == nearest_node)
          //   continue;
          // if (n == node)
          //   continue;
          // cost of near node
          double cost_to_near = costToNode(n);
          // std::cout<<"c:"<<c<<", cost to near:"<<cost_to_near<<", cost to node:"<<cost_to_node<<",conn cost:"<<std::get<6>(conn_data)<<std::endl;
          // if near node is not better than nearest node, skip
          if (cost_to_near >= cost_to_node)
            continue;
          // JF-for time-avoidance, cost function should return total time to reach node from start
          // double n_time = 0;
          double cost_near_to_node = std::get<6>(conn_data);
          // //JF - don't want to add costs for time-avoidance

          // // check for collisions between robot and real-time obstacles at configs along path from parent to node
          // // JF - need to ensure the predicted obstacles are not part of this planning scene
          // if (!checker_->checkPath(n->getConfiguration(), node->getConfiguration()))
          //   continue;

          // make a new connect from better parent to node
          ConnectionPtr conn = std::make_shared<Connection>(n, node);
          // edge cost is l2 distance or time to reach new node
          conn->setParentTime(std::get<2>(conn_data));
          conn->setAvoidIntervals(std::get<3>(conn_data), std::get<4>(conn_data), std::get<5>(conn_data));
          conn->setMinTime(inv_max_speed_, min_accel_time);
          if (1) {
            double n_time;
            std::vector<Eigen::Vector3f> avoid_ints;
            float last_pass_time;
            float min_human_dist;
            double test_cost = metrics_->cost(n, node, n_time, avoid_ints, last_pass_time, min_human_dist);
            std::cout<<"cfgs:";
            std::cout<<n->getConfiguration().transpose()<<"->"<<node->getConfiguration().transpose()<<std::endl;;
            std::cout<<"truth:";
            for (int a=0;a<avoid_ints.size();a++) std::cout<<avoid_ints[a].transpose()<<";";
            std::cout<<std::endl;
            std::vector<Eigen::Vector3f> est_avoid_ints = std::get<3>(conn_data);
            std::cout<<"estimate:";
            for (int a=0;a<est_avoid_ints.size();a++) std::cout<<est_avoid_ints[a].transpose()<<";";
            std::cout<<std::endl;
          }
          // std::cin.ignore();
          conn->add();
          conn->setCost(cost_near_to_node);
          if (((cost_near_to_node) >= cost_to_node))
          {
            conn->removeCache();
            continue;
          }
          else
          {
            // ROS_INFO_STREAM("node parents parents:"<<node->parent_connections_.size());
            if (node->parent_connections_.size()>1) node->parent_connections_.at(0)->removeCache();
          }
          

          // JF - if node cost is l2 distance between parent and node, then sum cost to parent with parent->node cost

          cost_to_node = cost_near_to_node;
          
          improved = true;
        }

        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        ROS_INFO_STREAM("STAP net Avoidance intervals " << time_span.count() << " seconds for "<<connection_datas.size()<<" configs");

      } else {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        for (const std::pair<double, NodePtr> &p : near_nodes)
        {
          const NodePtr &n = p.second; // get near node from pair
          // std::cout<<"near node:"<<*n<<std::endl;
          if (n == goal_node_)
            continue;
          // check to prevent a node from becoming its own parent
          if (n == nearest_node)
            continue;
          if (n == node)
            continue;
          // std::cout<<"near node:"<<*n<<std::endl;
          // if (n == goal_node_) {
          //   PATH_COMMENT_STREAM("goal node rewire "<<goal_node_->getConfiguration());
          // }
          // PATH_COMMENT_STREAM("test.........................");
          // cost of near node
          // ROS_INFO_STREAM("rewire"<<*node);
          // ROS_INFO_STREAM("to"<<*n);
          double cost_to_near = costToNode(n);
          // ROS_INFO_STREAM("rewire"<<goal_node_<<" 2.1"<<*goal_node_);
          // ROS_INFO_STREAM("num nodes:"<<getNodes().size());
          // if near node is not better than nearest node, skip
          // ROS_INFO_STREAM("cost_to_node:" << cost_to_node<< ", cost_to_near:" << cost_to_near);
          if (cost_to_near >= cost_to_node)
            continue;
          // JF-for time-avoidance, cost function should return total time to reach node from start
          double n_time = 0;
          double cost_near_to_node;
          std::vector<Eigen::Vector3f> avoid_ints;
          float min_human_dist;
          float last_pass_time;
          if (time_avoid_)
          {
            cost_near_to_node = metrics_->cost(n, node, n_time, avoid_ints, last_pass_time, min_human_dist);
          }
          else
          {
            cost_near_to_node = metrics_->cost(n, node);
          }
          // //JF - don't want to add costs for time-avoidance
          // if (cummulative_cost_) {
          // } else {
          //   if (cost_near_to_node >= cost_to_node)
          //     continue;
          // }

          // ROS_INFO_STREAM("Anew cost:" << cost_node_to_near + cost_to_node << ", old cost:" << cost_to_node<<","<<cost_near_to_node);
          if ((!time_avoid_) && (((cost_near_to_node + cost_to_near) >= cost_to_node)))
            continue;
          // check for collisions between robot and real-time obstacles at configs along path from parent to node
          // JF - need to ensure the predicted obstacles are not part of this planning scene
          //  if ((n!=goal_node_)||(nodes_->l2_dist(n,goal_node_->getConfiguration())>2)) {
          // ROS_INFO_STREAM("Cnew cost:" << cost_node_to_near + cost_to_node << ", old cost:" << cost_to_node);
          if (!checker_->checkPath(n->getConfiguration(), node->getConfiguration()))
            continue;
          // }
          // a better parent has been found and doesn't cause collision
          // remove old parent
          if (!time_avoid_)
            node->parent_connections_.at(0)->remove();
          // ROS_INFO_STREAM("Bnew cost:" << cost_node_to_near + cost_to_node << ", old cost:" << cost_to_node);
          // make a new connect from better parent to node
          ConnectionPtr conn = std::make_shared<Connection>(n, node);
          // edge cost is l2 distance or time to reach new node
          if (time_avoid_)
          {
            conn->setParentTime(n_time);
            conn->setAvoidIntervals(avoid_ints, last_pass_time, min_human_dist);
            conn->setMinTime(inv_max_speed_, min_accel_time);
          }
          conn->add();
          conn->setCost(cost_near_to_node);
          // if (((cost_to_near + cost_near_to_node) >= cost_to_node)) {
          if (time_avoid_)
          {
            if (((cost_near_to_node) >= cost_to_node))
            {
              conn->removeCache();
              continue;
            }
            else
            {
              // ROS_INFO_STREAM("node parents parents:"<<node->parent_connections_.size());
              if (node->parent_connections_.size()>1) node->parent_connections_.at(0)->removeCache();
            }
          }
          // std::cout<<"cost to node:"<<cost_to_node<<", new cost:"<<cost_to_near + cost_near_to_node<<std::endl;
          // std::cout<<"from node:"<<*n<<std::endl;
          // std::cout<<*node<<std::endl;
          // not better
          // {
          //   conn->remove();
          //   continue;
          // } else {

          // }
          // PATH_COMMENT_STREAM("adding connection3:"<<cost_near_to_node<<", "<<n_time<<","<<avoid_ints.size());
          // for (int i=0;i<avoid_ints.size();i++) {
          //   std::cout<<avoid_ints[i].transpose()<<std::endl;
          // }
          // PATH_COMMENT_STREAM("end of ints");

          // PATH_COMMENT_STREAM("adding connection2:"<<cost_near_to_node<<", "<<n_time<<","<<avoid_ints.size());

          // nearest_node = n; PER ME NON CI VA
          // JF - if node cost is l2 distance between parent and node, then sum cost to parent with parent->node cost
          //  if (cummulative_cost_) {
          //  cost_to_node = cost_to_near + cost_near_to_node;
          if (time_avoid_)
          {
            cost_to_node = cost_near_to_node;
          }
          else
          {
            cost_to_node = cost_near_to_node + cost_to_near;
          }
          // node->min_time = cost_to_node;
          // } else { //JF - else node cost is total time to reach node
          //   cost_to_node = cost_near_to_node;
          // }
          improved = true;
        }


        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        ROS_INFO_STREAM("Avoidance intervals " << time_span.count() << " seconds for "<<near_nodes.size()<<" configs");
      }
    }
    // std::cout<<"rewire parent hass:"<<node->parent_connections_.size()<<" parents\n";
    if (node->parent_connections_.empty())
    {
      std::cout << *node << std::endl;
    }
    // std::cout<<near_nodes.size()<<std::endl;

    if (rewire_children)
    {
      std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

      // ROS_INFO_STREAM("goal:" << goal_node_->getConfiguration().transpose());
      // ROS_INFO("try to find a better child between %zu nodes", near_nodes.size());
      for (const std::pair<double, NodePtr> &p : near_nodes)
      {
        const NodePtr &n = p.second;
        if (n == node)
          continue;
        // if (n == goal_node_)
          // ROS_INFO_STREAM("the goal is a near node");
        // if (n != root_)
          // ROS_INFO_STREAM("nearest child:" << n->getConfiguration().transpose() << "," << n->parent_connections_[0]->getCost() << "," << n->parent_connections_[0]->getParent()->getConfiguration().transpose());
        // ROS_INFO_STREAM("near node:"<<n->getConfiguration().transpose());
        // l2 distance or time to reach node n
        // ROS_INFO_STREAM("rewire"<<*n);
        // ROS_INFO_STREAM("to"<<*node);
        double cost_to_near = std::numeric_limits<double>::infinity();
        // ROS_INFO_STREAM("rewire3"<<*n);
        if (time_avoid_)
        {
          if ((n != goal_node_) || (!goal_node_->parent_connections_.empty()))
            cost_to_near = costToNode(n);
        }
        else
        {
          cost_to_near = costToNode(n);
        }
        // ROS_INFO_STREAM("rewire3.1");
        // node can not be a better parent to near node
        // ROS_INFO_STREAM("cost_to_node:" << cost_to_node << ", cost_to_near:" << cost_to_near);
        if (cost_to_node >= cost_to_near)
          continue;
        // std::vector<NodePtr> all_nodes = getNodes();
        // if (std::find(all_nodes.begin(),all_nodes.end(), goal_node_)==all_nodes.end()) ROS_INFO_STREAM("goal nodes is not in tree");
        // if (n == goal_node_) {
        //   PATH_COMMENT_STREAM("goal node rewire "<<cost_to_near<<","<<goal_node_->getConfiguration().transpose());
        // }
        // JF - if standard cost fn, get l2 distance between nodes
        double node_time = 0;
        double cost_node_to_near;

        float min_human_dist;
        std::vector<Eigen::Vector3f> avoid_ints;
        float last_pass_time;
        if (!time_avoid_)
        {
          cost_node_to_near = metrics_->cost(node->getConfiguration(), n->getConfiguration());
        }
        else
        { // JF - else get time to reach node n via node
          cost_node_to_near = metrics_->cost(node, n, node_time, avoid_ints, last_pass_time, min_human_dist);
        }
        // if the cost to reach n via node is not less than cost to reach n via n.parent, then skip
        //  if ((cost_to_node + cost_node_to_near) >= cost_to_near)
        //    continue;
        if (time_avoid_)
        {
          if ((cost_node_to_near) >= cost_to_near)
            continue;
        }
        else
        {
          // ROS_INFO_STREAM("new cost:" << cost_node_to_near + cost_to_node << ", old cost:" << cost_to_near << ", conn cost:" << cost_node_to_near);
          if ((cost_node_to_near + cost_to_node) >= cost_to_near)
            continue;
        }
        // node could still be a better parent to n, check for collision between node and n
        // if ((n != goal_node_) || (nodes_->l2_dist(n, goal_node_->getConfiguration()) > 2))
        // {
          if (!checker_->checkPath(node->getConfiguration(), n->getConfiguration()))
            continue;
        // }

        // ROS_INFO_STREAM("node parents rewire child:"<<node->parent_connections_.size());
        // node is a better parent for n and path is collision free, remove old n.parent
        if (!n->parent_connections_.empty())
        {
          // when removing a parent, note that the parent node could potentially be a parent of this node again
          if (time_avoid_)
          {
            n->parent_connections_.at(0)->removeCache();
          }
          else
          {
            n->parent_connections_.at(0)->remove();
          }
          // if (n==goal_node_) {
          //   PATH_COMMENT_STREAM("GOAL NODE REWIRE:"<<goal_node_->potential_parent_connections_.size());
          // }
        }
        // std::cout<<"Test\n";
        // make new connection between node and n
        ConnectionPtr conn = std::make_shared<Connection>(node, n);
        if (time_avoid_)
        {
          conn->setParentTime(node_time);
          conn->setAvoidIntervals(avoid_ints, last_pass_time, min_human_dist);
          conn->setMinTime(inv_max_speed_, min_accel_time);
        }
        conn->add();
        conn->setCost(cost_node_to_near);
        // if (n == goal_node_)
        //   ROS_INFO_STREAM("goal node improvement");

        // loop over near nodes a second time
        //  for (const std::pair<double,NodePtr>& p2:near_nodes) {
        //  const NodePtr& n_p = p2.second;
        // std::cout<<"test2\n";

        if (time_avoid_)
        {
          // ROS_INFO_STREAM("here");
          rewireNearToTheirChildren(n, 0);
          // ROS_INFO_STREAM("here1");
          rewireNearToBetterParents(n);
          // ROS_INFO_STREAM("here2");
          improved = !goal_node_->getParents().empty();
        }
        else
        {
          improved = true;
        }
        // should i call rewire_only on all child nodes of node n?

        // PATH_COMMENT_STREAM("rewired connection:"<<cost_node_to_near<<", "<<node_time<<","<<avoid_ints.size());
        // for (int i=0;i<avoid_ints.size();i++) {
        //   std::cout<<avoid_ints[i].transpose()<<std::endl;
        // }
        // PATH_COMMENT_STREAM("end of ints");
      }

      std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
      ROS_INFO_STREAM("children " << time_span.count() << " seconds for "<<near_nodes.size());
    }
    // std::cout<<"rewire child hass:"<<node->parent_connections_.size()<<" parents\n";

    return improved;
  }

  void Tree::rewireNearToTheirChildren(NodePtr n, int i)
  {
    if (i > 2)
      return;
    double cost_to_n = costToNode(n);
    // std::cout<<"rewire n has:"<<n->child_connections_.size()<<" children\n";
    for (ConnectionPtr conn : n->child_connections_)
    {
      NodePtr n_child = conn->getChild();
      if (n==n_child) {
        continue;
      }
      double cost_to_child = costToNode(n_child);
      std::vector<Eigen::Vector3f> avoid_ints;
      double node_time = 0;
      float last_pass_time;
      float min_human_dist;
      double cost_n_to_child = metrics_->cost(n, n_child, node_time, avoid_ints, last_pass_time, min_human_dist);
      if (cost_n_to_child >= cost_to_child)
        continue;
      // if (!checker_->checkPath(n_p->getConfiguration(), n->getConfiguration())) continue;
      // ROS_INFO_STREAM("node parents rewire child1:"<<n_child->parent_connections_.size());
      if (!n_child->parent_connections_.empty())
      {
        n_child->parent_connections_.at(0)->removeCache();
      }
      // ConnectionPtr conn = std::make_shared<Connection>(n, n_child);
      conn->setParentTime(node_time);
      conn->setAvoidIntervals(avoid_ints, last_pass_time, min_human_dist);
      conn->setMinTime(inv_max_speed_, min_accel_time);
      conn->add();
      conn->setCost(cost_n_to_child);
      rewireNearToTheirChildren(n_child, i + 1);
      // std::cout<<"rewire near to children, parent has:"<<conn->getParent()->parent_connections_.size()<<" parents\n";
    }
  }

  void Tree::rewireNearToBetterParents(NodePtr n)
  {
    double cost_to_n = costToNode(n);
    // std::cout<<"rewire n has:"<<n->potential_parent_connections_.size()<<" potential parents\n";
    for (int i = 0; i < n->potential_parent_connections_.size(); i++)
    {
      ConnectionPtr conn;
      try
      {
        conn = n->potential_parent_connections_[i];
      }
      catch (...)
      {
        continue;
      }
      if (!conn)
        continue;
      if (!conn->getAdded())
        continue;
      if (conn == n->parent_connections_.front())
        continue;
      // std::cout<<"test\n";
      // std::cout<<*conn;
      NodePtr n_parent = conn->getParent();
      // std::cout<<"test2\n";
      double cost_to_parent = costToNode(n_parent);
      std::vector<Eigen::Vector3f> avoid_ints;
      double node_time = 0;
      float last_pass_time;
      float min_human_dist;
      double cost_parent_to_n = metrics_->cost(n_parent, n, node_time, avoid_ints, last_pass_time, min_human_dist);
      if (cost_parent_to_n >= cost_to_n)
        continue;
      // if (!checker_->checkPath(n_p->getConfiguration(), n->getConfiguration())) continue;
      // ROS_INFO_STREAM("node parents rewire child2:"<<n->parent_connections_.size());
      if (!n->parent_connections_.empty())
      {
        n->parent_connections_.at(0)->removeCache();
      }
      // ConnectionPtr conn = std::make_shared<Connection>(n, n_child);
      conn->setParentTime(node_time);
      conn->setAvoidIntervals(avoid_ints, last_pass_time, min_human_dist);
      conn->setMinTime(inv_max_speed_, min_accel_time);
      conn->add();
      conn->setCost(cost_parent_to_n);
      // std::cout<<"rewire near to parents, parent has:"<<conn->getParent()->parent_connections_.size()<<" parents\n";
    }
  }

  void Tree::rewireTimeAvoidCheckChildren(NodePtr n, int iterations)
  {
    if (iterations > 10)
      return;
    // n would be children of a node just rewired
    // loop over all connections from n to child nodes
    for (ConnectionPtr conn : n->child_connections_)
    {
      double prev_conn_cost = conn->getCost();
      NodePtr child_node = conn->getChild();

      std::vector<Eigen::Vector3f> avoid_ints = conn->getAvoidIntervals();
      float last_pass_time = conn->getLPT();
      bool success = true;

      // PATH_COMMENT_STREAM("dist_new:"<<dist_new);

      double time_new = conn->getMinTime();
      // PATH_COMMENT_STREAM("time_new:"<<time_new);
      double node_time_new = time_new;
      // get cost of parent
      // requires extended node data
      double c_near = 0.0;
      // if (parent.min_time == std::numeric_limits<double>::infinity()) {
      //     c_near = parent->parent_connections_.at(0)->getCost(); //connections store cost to node at end of connection
      // } else {
      //     c_near = parent.min_time;
      // }
      if (!n->parent_connections_.empty())
      {
        c_near = n->min_time;
      }
      double near_time = c_near;
      // get min cost to get to new node from near parent
      double c_new = c_near + time_new;

      // if node n connection parenttime after rewiring allows the childred of node n to be reached sooner,
      // then update timing of connections between n and n's children;
      // if avoidance intervals, loop over avoidance intervals to determine soonest time of passage from parent to new node
      if (c_new > last_pass_time)
      {
        success = false;
        // break;
      }
      if (!avoid_ints.empty() && success)
      {
        bool found_avoid = false;
        double tmp_time = 0.0;
        double tmp_c_new = c_new;
        double t_pad_ = 0;
        for (int r = 0; r < avoid_ints.size(); r++)
        {
          // if time to reach new node is within an avoidance interval +/- padding then set time to reach new node to the end of the avoidance interval
          // repeat check with the time to reach the parent node when parent time is increased
          // ensuring the entire path from parent to new node is not in an avoidance interval
          if (((avoid_ints[r][0] - t_pad_ < tmp_c_new) && (tmp_c_new < avoid_ints[r][1] + t_pad_)) || ((avoid_ints[r][0] - t_pad_ < near_time) && (near_time < avoid_ints[r][1] + t_pad_)))
          {
            tmp_time = avoid_ints[r][1] + node_time_new + t_pad_;
            if (tmp_time > tmp_c_new)
            {
              tmp_c_new = tmp_time;
              near_time = avoid_ints[r][1] + t_pad_;
            }
            found_avoid = true;
          }
          else if (found_avoid)
          {
            c_new = tmp_c_new;
            found_avoid = false;
            break;
          }
        }
        if (found_avoid)
        {
          if (avoid_ints.back()[2])
          {
            c_new = std::numeric_limits<double>::infinity();
            near_time = std::numeric_limits<double>::infinity();
          }
          else
          {
            c_new = avoid_ints.back()[1] + node_time_new + t_pad_;
            near_time = avoid_ints.back()[1] + t_pad_;
          }
        }
        c_new = std::max(c_new, tmp_c_new);
      }
      // PATH_COMMENT_STREAM("cnear:"<<c_near<<", c new:"<<c_near + time_new<<", c_new:"<<c_new<<", parent min time:"<<parent->min_time<<", last pass time:"<<last_pass_time<<","<<avoid_ints.size());

      // return inf cost if not success
      if (!success)
      {
        c_new = std::numeric_limits<double>::infinity();
      }

      // PATH_COMMENT_STREAM("previous conn cost:"<<prev_conn_cost<<", new conn cost:"<<c_new-n->min_time);
      conn->setParentTime(near_time);
      conn->setCost(c_new - n->min_time);

      rewireTimeAvoidCheckChildren(child_node, iterations + 1);
    }
  }

  // void Tree::rewireTimeAvoidCheckChildren(NodePtr n, int iterations) {
  //   if (iterations>10) return;
  //   //n would be children of a node just rewired
  //   //loop over all connections from n to child nodes
  //   for (ConnectionPtr conn:n->potential_parent_connections_) {
  //     NodePtr child_node = conn->getChild();
  //     ConnectionPtr curr_child_conn = child_node->parent_connections_.at(0);
  //     NodePtr cur_child_parent = curr_child_conn->getParent();
  //     double prev_conn_time = curr_child_conn->getParent()->min_time + curr_child_conn->getCost();

  //     std::vector<Eigen::Vector3f> avoid_ints = conn->getAvoidIntervals();
  //     float last_pass_time = conn->getLPT();
  //     bool success = true;

  //     // PATH_COMMENT_STREAM("dist_new:"<<dist_new);

  //     double time_new = conn->getMinTime();
  //     // PATH_COMMENT_STREAM("time_new:"<<time_new);
  //     double node_time_new = time_new;
  //     //get cost of parent
  //     //requires extended node data
  //     double c_near = n->min_time;

  //     double near_time = c_near;
  //     // get min cost to get to new node from near parent
  //     double c_new = c_near + time_new;

  //     //if node n connection parenttime after rewiring allows the childred of node n to be reached sooner,
  //     //then update timing of connections between n and n's children;
  //     //if avoidance intervals, loop over avoidance intervals to determine soonest time of passage from parent to new node
  //     if (c_new>last_pass_time) {
  //         success = false;
  //         // break;
  //     }
  //     if (!avoid_ints.empty() && success){
  //         bool found_avoid = false;
  //         double tmp_time = 0.0;
  //         double tmp_c_new = c_new;
  //         double t_pad_=0;
  //         for (int r=0;r<avoid_ints.size();r++) {
  //             //if time to reach new node is within an avoidance interval +/- padding then set time to reach new node to the end of the avoidance interval
  //             //repeat check with the time to reach the parent node when parent time is increased
  //             //ensuring the entire path from parent to new node is not in an avoidance interval
  //             if (((avoid_ints[r][0]-t_pad_<tmp_c_new) && (tmp_c_new<avoid_ints[r][1]+t_pad_)) || ((avoid_ints[r][0]-t_pad_<near_time) && (near_time<avoid_ints[r][1]+t_pad_))) {
  //                 tmp_time = avoid_ints[r][1]+node_time_new + t_pad_;
  //                 if (tmp_time>tmp_c_new) {
  //                     tmp_c_new = tmp_time;
  //                     near_time = avoid_ints[r][1]+t_pad_;
  //                 }
  //                 found_avoid = true;
  //             } else if (found_avoid) {
  //                 c_new = tmp_c_new;
  //                 found_avoid = false;
  //                 break;
  //             }
  //         }
  //         if (found_avoid) {
  //             if (avoid_ints.back()[2]) {
  //               c_new = std::numeric_limits<double>::infinity();
  //               near_time = std::numeric_limits<double>::infinity();
  //             } else {
  //               c_new = avoid_ints.back()[1]+node_time_new + t_pad_;
  //               near_time = avoid_ints.back()[1]+t_pad_;
  //             }
  //         }
  //         c_new = std::max(c_new,tmp_c_new);
  //     }
  //       // PATH_COMMENT_STREAM("cnear:"<<c_near<<", c new:"<<c_near + time_new<<", c_new:"<<c_new<<", parent min time:"<<parent->min_time<<", last pass time:"<<last_pass_time<<","<<avoid_ints.size());

  //     //return inf cost if not success
  //     if (!success) {
  //         c_new = std::numeric_limits<double>::infinity();
  //     }

  //     // PATH_COMMENT_STREAM("previous conn cost:"<<prev_conn_cost<<", new conn cost:"<<c_new-n->min_time);
  //     conn->setParentTime(near_time);
  //     conn->setCost(c_new-n->min_time);

  //     // if the new timing of the parent node has made this connection to the child better than the connection between child and previous parent, then rewire.
  //     std::vector<NodePtr> parent_train = getNodesToNode(child_node);
  //     std::vector<NodePtr> child_train = getNodesFromNode(child_node);
  //     for (NodePtr np:parent_train) {
  //       if (std::find(child_train.begin(),child_train.end(),np)!=child_train.end()){
  //         ROS_INFO_STREAM("is broke");
  //         return;
  //       }
  //     }

  //     if (c_new<prev_conn_time) {
  //       curr_child_conn->remove();
  //       child_node->addParentConnection(conn);
  //       rewireTimeAvoidCheckChildren(child_node,iterations+1);
  //     }

  //   }
  // }

  bool Tree::rewireOnlyWithPathCheck(NodePtr &node, std::vector<ConnectionPtr> &checked_connections, double r_rewire, const int &what_rewire)
  {

    if (what_rewire > 2 || what_rewire < 0)
    {
      ROS_ERROR("what_rewire parameter should be 0,1 or 2");
      assert(0);
      return false;
    }

    bool rewire_parent;
    bool rewire_children;

    switch (what_rewire)
    {
    case 0: // default is 0 from function declaration in header
      rewire_parent = true;
      rewire_children = true;
      break;
    case 1:
      rewire_parent = true;
      rewire_children = false;
      break;
    case 2:
      rewire_parent = false;
      rewire_children = true;
      break;
    }

    // new node, no parents
    if (node->getParents().size() == 0)
      rewire_parent = false;
    // get nodes in neighborhood of node
    std::multimap<double, NodePtr> near_nodes = near(node, r_rewire);

    // validate connections to node
    double cost_to_node;
    if (!checkPathToNode(node, checked_connections))
      cost_to_node = std::numeric_limits<double>::infinity();
    else
      cost_to_node = costToNode(node);

    bool improved = false;

    // by default, this that if the node is not new, then it can be rewired to a better parent
    if (rewire_parent)
    {
      // ROS_DEBUG("try to find a better parent between %zu nodes", near_nodes.size());
      NodePtr nearest_node = node->getParents().at(0);
      for (const std::pair<double, NodePtr> &p : near_nodes)
      {
        const NodePtr &n = p.second;
        if (n == nearest_node)
          continue;
        if (n == node)
          continue;

        double cost_to_near = costToNode(n);

        if (cost_to_near >= cost_to_node)
          continue;

        double cost_near_to_node = metrics_->cost(n, node);

        if ((cost_to_near + cost_near_to_node) >= cost_to_node)
          continue;

        if (!checkPathToNode(n, checked_connections)) // validate connections to n
          continue;

        if (!checker_->checkPath(n->getConfiguration(), node->getConfiguration()))
          continue;

        node->parent_connections_.at(0)->remove();

        ConnectionPtr conn = std::make_shared<Connection>(n, node);
        conn->setCost(cost_near_to_node);
        conn->add();
        checked_connections.push_back(conn);

        //      nearest_node = n; //PER ME NON CI VA
        cost_to_node = cost_to_near + cost_near_to_node;
        improved = true;
      }
    }

    if (cost_to_node == std::numeric_limits<double>::infinity())
      rewire_children = false;

    if (rewire_children)
    {
      // ROS_DEBUG("try to find a better child between %zu nodes", near_nodes.size());
      for (const std::pair<double, NodePtr> &p : near_nodes)
      {
        const NodePtr &n = p.second;
        if (n == node)
          continue;
        if (n->getParents().size() == 0)
        {

          continue;
        }

        double cost_to_near = costToNode(n);
        bool checked = false;

        if (cost_to_node >= cost_to_near)
        {
          if (checkPathToNode(n, checked_connections)) // if path to n is free, cost_to_node >= cost_to_near is really true
            continue;

          checked = true;
        }

        double cost_node_to_near = metrics_->cost(node, n);
        if ((cost_to_node + cost_node_to_near) >= cost_to_near)
        {
          if (checked)
            continue;
          else
          {
            if (checkPathToNode(n, checked_connections)) // if path to n is free, cost_to_node +cost_node_to_near >= cost_to_near is really true
              continue;
          }
        }

        if (!checker_->checkPath(node->getConfiguration(), n->getConfiguration()))
          continue;

        n->parent_connections_.at(0)->remove();

        ConnectionPtr conn = std::make_shared<Connection>(node, n);
        conn->setCost(cost_node_to_near);
        conn->add();

        checked_connections.push_back(conn);

        improved = true;
      }
    }
    return improved;
  }

  bool Tree::rewireK(const Eigen::VectorXd &configuration)
  {
    NodePtr new_node;
    if (!extend(configuration, new_node))
    {
      return false;
    }
    std::multimap<double, NodePtr> near_nodes = nearK(new_node);
    NodePtr nearest_node = new_node->getParents().at(0);
    double cost_to_new = costToNode(new_node);

    bool improved = false;

    // ROS_DEBUG("try to find a better parent between %zu nodes", near_nodes.size());
    for (const std::pair<double, NodePtr> &p : near_nodes)
    {
      const NodePtr &node = p.second;

      if (node == nearest_node)
        continue;
      if (node == new_node)
        continue;

      double cost_to_near = costToNode(node);

      if (cost_to_near >= cost_to_new)
        continue;

      double cost_near_to_new = metrics_->cost(node, new_node);

      if ((cost_to_near + cost_near_to_new) >= cost_to_new)
        continue;

      if (!checker_->checkPath(node->getConfiguration(), new_node->getConfiguration()))
        continue;

      new_node->parent_connections_.at(0)->remove();

      ConnectionPtr conn = std::make_shared<Connection>(node, new_node);
      conn->setCost(cost_near_to_new);
      conn->add();
      nearest_node = node;
      cost_to_new = cost_to_near + cost_near_to_new;
      improved = true;
    }

    // ROS_DEBUG("try to find a better child between %zu nodes", near_nodes.size());
    for (const std::pair<double, NodePtr> &p : near_nodes)
    {
      const NodePtr &n = p.second;
      if (n == new_node)
        continue;

      double cost_to_near = costToNode(n);
      if (cost_to_new >= cost_to_near)
        continue;

      double cost_new_to_near = metrics_->cost(new_node->getConfiguration(), n->getConfiguration());
      if ((cost_to_new + cost_new_to_near) >= cost_to_near)
        continue;

      if (!checker_->checkPath(new_node->getConfiguration(), n->getConfiguration()))
        continue;

      n->parent_connections_.at(0)->remove();
      ConnectionPtr conn = std::make_shared<Connection>(new_node, n);
      conn->setCost(cost_new_to_near);
      conn->add();

      improved = true;
    }

    return improved;
  }

  bool Tree::rewireWithPathCheck(const Eigen::VectorXd &configuration, std::vector<ConnectionPtr> &checked_connections, double r_rewire, NodePtr &new_node)
  {
    ConnectionPtr new_conn;
    if (!extend(configuration, new_node, new_conn))
    {
      return false;
    }

    checked_connections.push_back(new_conn);

    return rewireOnlyWithPathCheck(new_node, checked_connections, r_rewire);
  }

  bool Tree::rewire(const Eigen::VectorXd &configuration, double r_rewire, NodePtr &new_node)
  {
    if (!extend(configuration, new_node))
    {
      // ROS_INFO_STREAM("fail:"<<*new_node);
      return false;
    }
    // ROS_INFO_STREAM("success:"<<new_node);
    // PATH_COMMENT_STREAM("rewire only");
    return rewireOnly(new_node, r_rewire);
  }

  bool Tree::rewire(const Eigen::VectorXd &configuration, double r_rewire)
  {
    NodePtr new_node;

    return rewire(configuration, r_rewire, new_node);
  }

  bool Tree::rewireToNode(const NodePtr &n, double r_rewire, NodePtr &new_node)
  {
    if (!extendToNode(n, new_node))
    {
      return false;
    }

    return rewireOnly(new_node, r_rewire);
  }

  bool Tree::rewireToNode(const NodePtr &n, double r_rewire)
  {
    NodePtr new_node;

    return rewireToNode(n, r_rewire, new_node);
  }

  std::multimap<double, NodePtr> Tree::near(const NodePtr &node, const double &r_rewire)
  {
    return nodes_->near(node->getConfiguration(), r_rewire);
  }

  std::multimap<double, NodePtr> Tree::nearK(const NodePtr &node)
  {
    return nearK(node->getConfiguration());
  }

  std::multimap<double, NodePtr> Tree::nearK(const Eigen::VectorXd &conf)
  {
    size_t k = std::ceil(k_rrt_ * std::log(nodes_->size() + 1));
    return nodes_->kNearestNeighbors(conf, k);
  }

  double Tree::costToNode(NodePtr node)
  {
    double cost = 0;
    if (!time_avoid_)
    {
      while (node != root_)
      {
        // ROS_INFO_STREAM("getting cost:"<<node);
        if (node->parent_connections_.size() != 1)
        {
          ROS_ERROR_STREAM("a tree node should have exactly a parent. this node has " << node->parent_connections_.size() << ": " << *node);
          // ROS_INFO_STREAM("node ptr "<<node<<", goal ptr"<<goal_node_);
          return std::numeric_limits<double>::infinity();
        }

        if (node->parent_connections_.at(0)->getParent() == node)
        {
          ROS_FATAL_STREAM("node " << node.get() << "=\n"
                                   << *node);
          ROS_FATAL_STREAM("to parent\n"
                           << *(node->parent_connections_.at(0)));
          ROS_FATAL("connection between the same node");
          assert(0);
        }
        cost += node->parent_connections_.at(0)->getCost();
        // ROS_INFO_STREAM(cost);
        node = node->parent_connections_.at(0)->getParent();
      }
    }
    else
    {
      if (node->parent_connections_.empty()) {
        cost = std::numeric_limits<double>::infinity();
      } else if (node != root_) {
          cost = node->parent_connections_.at(0)->getCost();
      }
    }
    return cost;
  }

  std::vector<ConnectionPtr> Tree::getConnectionToNode(NodePtr node)
  {
    std::vector<ConnectionPtr> connections;

    while (node != root_)
    {
      // ROS_INFO_STREAM(*node);
      if (node->parent_connections_.size() != 1)
      {
        ROS_ERROR("a tree node should have only a parent");
        ROS_ERROR_STREAM("node \n"
                         << *node);

        ROS_INFO_STREAM("current root " << root_);
        ROS_INFO_STREAM("node " << node);

        assert(0);
      }
      connections.push_back(node->parent_connections_.at(0));
      node = node->parent_connections_.at(0)->getParent();
    }
    std::reverse(connections.begin(), connections.end());

    return connections;
  }
  std::vector<NodePtr> Tree::getNodesToNode(NodePtr node)
  {
    std::vector<NodePtr> nodes;
    nodes.push_back(node);

    while (node != root_)
    {
      if (node->parent_connections_.size() != 1)
      {
        ROS_ERROR("a tree node should have only a parent");
        ROS_ERROR_STREAM("node \n"
                         << *node);

        ROS_INFO_STREAM("current root " << root_);
        ROS_INFO_STREAM("node " << node);

        assert(0);
      }
      node = node->parent_connections_.at(0)->getParent();
      nodes.push_back(node);
    }
    // std::reverse(connections.begin(), connections.end());

    return nodes;
  }

  std::vector<NodePtr> Tree::getNodesFromNode(NodePtr node)
  {
    std::vector<NodePtr> nodes;
    for (ConnectionPtr child_conn : node->child_connections_)
    {
      nodes.push_back(child_conn->getChild());
      std::vector<NodePtr> tmp = getNodesFromNode(child_conn->getChild());
      nodes.insert(nodes.end(), tmp.begin(), tmp.end());
    }

    return nodes;
  }

  void Tree::addNode(const NodePtr &node, const bool &check_if_present)
  {
    if (!check_if_present || !isInTree(node))
      nodes_->insert(node);
  }

  void Tree::removeNode(const NodePtr &node)
  {
    node->disconnect();
    nodes_->deleteNode(node);
  }

  bool Tree::keepOnlyThisBranch(const std::vector<ConnectionPtr> &connections)
  {
    if (connections.size() == 0)
      return false;

    std::vector<NodePtr> branch_nodes;
    branch_nodes.push_back(connections.at(0)->getParent());
    for (const ConnectionPtr &conn : connections)
      branch_nodes.push_back(conn->getChild());

    nodes_->disconnectNodes(branch_nodes);
    NearestNeighborsPtr nodes;
    for (NodePtr &n : branch_nodes)
      nodes->insert(n);
    nodes_ = nodes;

    return true;
  }

  bool Tree::addBranch(const std::vector<ConnectionPtr> &connections)
  {
    if (connections.size() == 0)
      return false;

    NodePtr start_node = connections.at(0)->getParent();
    if (!isInTree(start_node))
    {
      ROS_ERROR("start node of the branch is not part of the tree");
      return false;
    }

    std::vector<NodePtr> branch_nodes;
    for (const ConnectionPtr &conn : connections)
      branch_nodes.push_back(conn->getChild());

    for (NodePtr &n : branch_nodes)
    {
      if (not nodes_->findNode(n))
        nodes_->insert(n);
    }
    return true;
  }

  bool Tree::addTree(TreePtr &additional_tree, const double &max_time)
  {
    if (not isInTree(additional_tree->getRoot()))
    {
      PATH_COMMENT_STREAM("root not in path?");
      NodePtr new_node;
      if (not connectToNode(additional_tree->getRoot(), new_node, max_time))
        return false;
    }

    std::vector<NodePtr> additional_nodes = additional_tree->getNodes();
    for (size_t inode = 1; inode < additional_nodes.size(); inode++)
    {
      addNode(additional_nodes.at(inode), false);
    }
    return true;
  }

  bool Tree::isInTree(const NodePtr &node)
  {
    return nodes_->findNode(node);
  }

  unsigned int Tree::purgeNodesOutsideEllipsoid(const SamplerPtr &sampler, const std::vector<NodePtr> &white_list)
  {
    if (nodes_->size() < maximum_nodes_)
      return 0;
    unsigned int removed_nodes = 0;

    purgeNodeOutsideEllipsoid(root_, sampler, white_list, removed_nodes);
    return removed_nodes;
  }

  unsigned int Tree::purgeNodesOutsideEllipsoids(const std::vector<SamplerPtr> &samplers, const std::vector<NodePtr> &white_list)
  {
    if (nodes_->size() < maximum_nodes_)
      return 0;
    unsigned int removed_nodes = 0;

    purgeNodeOutsideEllipsoids(root_, samplers, white_list, removed_nodes);
    return removed_nodes;
  }

  void Tree::purgeNodeOutsideEllipsoid(NodePtr &node,
                                       const SamplerPtr &sampler,
                                       const std::vector<NodePtr> &white_list,
                                       unsigned int &removed_nodes)
  {
    assert(node);

    // check if it is in the admissible informed set
    if (sampler->inBounds(node->getConfiguration()))
    {
      // if node is inside the admissible set, check its successors
      std::vector<NodePtr> successors;
      successors = node->getChildren();

      for (NodePtr &n : successors)
      {
        assert(n.get() != node.get());
        purgeNodeOutsideEllipsoid(n, sampler, white_list, removed_nodes);
      }
    }
    else
    {
      // if node is outside the admissible set, remove it and its successors if they are not in the white list.
      if (std::find(white_list.begin(), white_list.end(), node) != white_list.end())
        return;
      purgeFromHere(node, white_list, removed_nodes);
    }
    return;
  }

  void Tree::purgeNodeOutsideEllipsoids(NodePtr &node,
                                        const std::vector<SamplerPtr> &samplers,
                                        const std::vector<NodePtr> &white_list,
                                        unsigned int &removed_nodes)
  {

    if (nodes_->size() < 0.5 * maximum_nodes_)
      return;
    assert(node);

    // check if it belongs to a admissible informed set or the white list
    bool inbound = std::find(white_list.begin(), white_list.end(), node) != white_list.end();
    for (const SamplerPtr &sampler : samplers)
      inbound = inbound || sampler->inBounds(node->getConfiguration());

    if (inbound)
    {
      // if node is inside the admissible set, check its successors
      std::vector<NodePtr> successors;
      successors = node->getChildren();

      for (NodePtr &n : successors)
      {
        assert(n.get() != node.get());
        purgeNodeOutsideEllipsoids(n, samplers, white_list, removed_nodes);
      }
    }
    else
    {
      purgeFromHere(node, white_list, removed_nodes);
    }
    return;
  }

  bool Tree::purgeFromHere(NodePtr &node)
  {
    std::vector<NodePtr> white_list;
    unsigned int removed_nodes;

    return purgeFromHere(node, white_list, removed_nodes);
  }

  bool Tree::purgeFromHere(NodePtr &node, const std::vector<NodePtr> &white_list, unsigned int &removed_nodes)
  {
    if (std::find(white_list.begin(), white_list.end(), node) != white_list.end())
    {
      return false;
    }
    assert(node);
    std::vector<NodePtr> successors;

    successors = node->getChildren();

    for (NodePtr &n : successors)
    {
      assert(n.get() != node.get());
      if (!purgeFromHere(n, white_list, removed_nodes))
        return false;
    }

    assert(node);
    node->disconnect();
    if (nodes_->deleteNode(node))
    {
      removed_nodes++;
    }
    return true;
  }

  void Tree::cleanTree()
  {
    std::vector<NodePtr> successors;
    std::vector<NodePtr> white_list;
    unsigned int removed_nodes;
    successors = root_->getChildren();
    for (NodePtr &n : successors)
    {
      if (isInTree(n))
        purgeFromHere(n, white_list, removed_nodes);
    }
  }

  void Tree::populateTreeFromNode(const NodePtr &node)
  {
    Eigen::VectorXd focus1 = node->getConfiguration();
    Eigen::VectorXd focus2 = node->getConfiguration();
    populateTreeFromNode(node, focus1, focus2, std::numeric_limits<double>::infinity());
  }

  void Tree::populateTreeFromNode(const NodePtr &node, const std::vector<NodePtr> &white_list)
  {
    Eigen::VectorXd focus1 = node->getConfiguration();
    Eigen::VectorXd focus2 = node->getConfiguration();
    populateTreeFromNode(node, focus1, focus2, std::numeric_limits<double>::infinity(), white_list);
  }

  void Tree::populateTreeFromNode(const NodePtr &node, const Eigen::VectorXd &focus1, const Eigen::VectorXd &focus2, const double &cost)
  {
    std::vector<NodePtr> white_list;
    populateTreeFromNode(node, focus1, focus2, std::numeric_limits<double>::infinity(), white_list);
  }

  void Tree::populateTreeFromNode(const NodePtr &node, const Eigen::VectorXd &focus1, const Eigen::VectorXd &focus2, const double &cost, const std::vector<NodePtr> &white_list)
  {
    if (not nodes_->findNode(node))
    {
      throw std::invalid_argument("node is not member of tree");
    }

    for (const NodePtr &n : node->getChildren())
    {
      std::vector<NodePtr>::const_iterator it = std::find(white_list.begin(), white_list.end(), n);
      if (it != white_list.end())
        continue;

      else
      {
        if (((n->getConfiguration() - focus1).norm() + (n->getConfiguration() - focus2).norm()) < cost)
        {
          nodes_->insert(n);
          populateTreeFromNode(n, focus1, focus2, cost, white_list);
        }
      }
    }
  }

  XmlRpc::XmlRpcValue Tree::toXmlRpcValue() const
  {
    XmlRpc::XmlRpcValue tree;
    XmlRpc::XmlRpcValue nodes;
    XmlRpc::XmlRpcValue connections;
    int iconn = 0;

    std::vector<NodePtr> nodes_vector = nodes_->getNodes();
    for (size_t inode = 0; inode < nodes_vector.size(); inode++)
    {
      const NodePtr &n = nodes_vector.at(inode);
      nodes[inode] = n->toXmlRpcValue();
      std::vector<NodePtr> dest;
      dest = n->getChildren();

      for (size_t idest = 0; idest < dest.size(); idest++)
      {
        for (size_t in2 = 0; in2 < nodes_vector.size(); in2++)
        {
          if (nodes_vector.at(in2) == dest.at(idest))
          {
            XmlRpc::XmlRpcValue connection;
            connection[0] = (int)inode;
            connection[1] = (int)in2;
            connections[iconn++] = connection;
            break;
          }
        }
      }
    }
    tree["nodes"] = nodes;
    tree["connections"] = connections;
    return tree;
  }

  std::ostream &operator<<(std::ostream &os, const Tree &tree)
  {
    os << "number of nodes = " << tree.nodes_->size() << std::endl;
    os << "root = " << *tree.root_;
    return os;
  }

  TreePtr Tree::fromXmlRpcValue(const XmlRpc::XmlRpcValue &x,
                                const double &max_distance,
                                const CollisionCheckerPtr &checker,
                                const MetricsPtr &metrics,
                                const bool &lazy)
  {
    if (not x.hasMember("nodes"))
    {
      ROS_ERROR("loading from XmlRpcValue a tree without 'nodes' field");
      return NULL;
    }
    if (not x.hasMember("connections"))
    {
      ROS_ERROR("loading from XmlRpcValue a tree without 'connections' field");
      return NULL;
    }

    XmlRpc::XmlRpcValue nodes = x["nodes"];
    XmlRpc::XmlRpcValue connections = x["connections"];
    if (nodes.getType() != XmlRpc::XmlRpcValue::Type::TypeArray)
    {
      ROS_ERROR("loading from XmlRpcValue a tree where 'nodes' is not an array");
      return NULL;
    }
    if (connections.getType() != XmlRpc::XmlRpcValue::Type::TypeArray)
    {
      ROS_ERROR("loading from XmlRpcValue a tree where 'connections' is not an array");
      return NULL;
    }
    NodePtr root = Node::fromXmlRpcValue(nodes[0]);
    if (not lazy)
    {
      if (not checker->check(root->getConfiguration()))
      {
        ROS_DEBUG("root is in collision");
        return NULL;
      }
    }
    assert(root);

    std::vector<NodePtr> nodes_vector(nodes.size());
    nodes_vector.at(0) = root;
    for (int inode = 1; inode < nodes.size(); inode++)
    {
      nodes_vector.at(inode) = Node::fromXmlRpcValue(nodes[inode]);
    }

    for (int iconn = 0; iconn < connections.size(); iconn++)
    {
      int in1 = connections[iconn][0];
      int in2 = connections[iconn][1];

      NodePtr &n1 = nodes_vector.at(in1);
      NodePtr &n2 = nodes_vector.at(in2);
      ConnectionPtr conn;
      conn = std::make_shared<Connection>(n1, n2);
      conn->setCost(metrics->cost(n1, n2));

      conn->add();
    }
    pathplan::TreePtr tree = std::make_shared<Tree>(root, max_distance, checker, metrics);
    for (int inode = 1; inode < nodes.size(); inode++)
    {
      tree->addNode(nodes_vector.at(inode), false);
    }

    if (not lazy)
    {
      tree->recheckCollision();
    }
    return tree;
  }

  bool Tree::changeRoot(const NodePtr &node)
  {
    if (not isInTree(node))
      return false;

    std::vector<ConnectionPtr> connections = getConnectionToNode(node);
    for (ConnectionPtr &conn : connections)
    {
      conn->flip();
    }

    root_ = node;

    return true;
  }

  bool Tree::recheckCollision()
  {
    return recheckCollisionFromNode(root_);
  }
  bool Tree::recheckCollisionFromNode(NodePtr &n)
  {
    std::vector<NodePtr> white_list;
    unsigned int removed_nodes;
    for (ConnectionPtr conn : n->child_connections_)
    {
      NodePtr n = conn->getChild();
      if (not checker_->checkConnection(conn))
      {
        purgeFromHere(n, white_list, removed_nodes);
        return false;
      }
      if (not recheckCollisionFromNode(n))
        return false;
    }
    return true;
  }

} // end namespace pathplan
