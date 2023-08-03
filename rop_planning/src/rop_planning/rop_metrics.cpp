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

#include <rop_planning/rop_metrics.h>


namespace pathplan
{

ROPMetrics::ROPMetrics(const Eigen::VectorXd& max_speed, const Eigen::VectorXd& max_acc, const double& nu):
  Metrics(),
  max_speed_(max_speed),
  max_acc_(max_acc),
  nu_(nu),
{

  name="rop metrics";

  urdf::Model robo_model;
  robo_model.initParam("robot_description");
  std::string base_frame_ = "world";
  std::string tool_frame = "tip";
  if (!nh.getParam("base_frame", base_frame_))
  {
    ROS_ERROR("%s/base_frame not defined", nh.getNamespace().c_str());
    throw std::invalid_argument("base_frame is not defined");
  }
  if (!nh.getParam("tool_frame", tool_frame))
  {
    ROS_ERROR("%s/tool_frame not defined", nh.getNamespace().c_str());
    throw std::invalid_argument("base_frame is not defined");
  }

  if (!nh.getParam("avoid_prob_threshold", prob_threshold))
  {
    ROS_WARN_STREAM("/avoid_prob_threshold not defined, using "<<prob_threshold);
  }
  
  if (!nh.getParam("links_for_rop", links_to_check))
  {
    ROS_ERROR("%s/links_for_rop not defined", nh.getNamespace().c_str());
    throw std::invalid_argument("links_for_rop is not defined");
  }  
  if (!nh.getParam("rop_resolution", m_resolution))
  {
    ROS_WARN_STREAM("/rop_resolution not defined, using "<<m_resolution);
  }
  
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  chain = rosdyn::createChain(robo_model, base_frame_, tool_frame, grav);
  
  link_names=chain->getLinksName();
  Eigen::VectorXd q;
  q.resize(link_names.size());
  q.setZero();
  m_transformations=chain->getTransformations(q);
  for (std::string link_name:links_to_check) {
    auto it = std::find(link_names.begin(),link_names.end(),link_name);
    Eigen::Affine3d t_tip = m_transformations.at(it);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> link_pts;
    link_pts.emplace_back(t_tip.translation());
    if (it!=link_names.begin()) {
      Eigen::Affine3d t_start = m_transformations.at(it-1);
      Eigen::Vector3d diff = t_tip.translation()-t_start.translation();
      int npts = std::ceil(diff.norm()/m_resolution);
      for (int i=1;i<npts;i++) {
        link_pts.emplace_back(t_start.translation()+double(i/npts)*diff);
      }
    }
    m_test_points.emplace_back(link_name,link_pts);
  }
  m_points.resize(m_test_points);

  if (!nh_.getParam("computation_step", step_))
  {
    ROS_ERROR("%s/computation_step not defined", nh_.getNamespace().c_str());
  }
  std::string opv_topic = "rop_opvs"
  if (!nh_.getParam("rop_opv_topic", opv_topic))
  {
    ROS_WARN_STREAM("/rop_opv_topic not defined, using "<<opv_topic);
  }

  WorkcellGrid();

  sub_opvs = nh_.subscribe<rop_planning::opv_array_msg>(opv_topic,1,&ROPMetrics::opvs_callback,this);
}

ROPMetrics::WorkcellGrid(void)
{
  
  std::vector<double> ws_lb_param;

  if (m_nh.getParam("workspace_lower_bounds_xyz",ws_lb_param))
  {
    for (int i=0;i<3;i++) workspace_lb[i] = ws_lb_param[i];
  } else {
    ROS_DEBUG("workspace_lower_bounds_xyz is not set, default={-1,-1,0.5}");
  }  

  std::vector<double> ws_ub_param;

  if (m_nh.getParam("workspace_upper_bounds_xyz",ws_ub_param))
  {
    for (int i=0;i<3;i++) workspace_ub[i] = ws_ub_param[i];
  } else {
    ROS_DEBUG("workspace_lower_bounds_xyz is not set, default={1,1,2.5}");
  }
  std::vector<int> npnt;
  for (int i=0;i<3;i++) {
    npnt.push_back(std::ceil((workspace_up[i]-workspace_lb[i])/m_resolution));
    assert(npnt.back()>0);
  }
  m_voxel_volume=m_resolution * m_resolution * m_resolution;
  m_inv_resolution=1/m_resolution;
  m_occupancy.resize(npnt[0],npnt[1],npnt[2]);
  m_occupancy.setOnes();
  m_npnt=npnt;
}

void ROPMetrics::opvs_callback(const rop_planning::opv_array_msg &msg)
{
  m_occupancy.setOnes();
  for (rop_planning::opv_msg opv:msg->opv_array) {
    //descritize OPV
    Eigen::Tensor<double,3> occup(m_npnt[0],m_npnt[1],m_npnt[2]);
    occup.setOnes();
    Eigen::Vector3d near_pt_1;
    for (int i=0;i<3;i++) near_pt_1[i]=opv.near_pt_1[i];
    Eigen::Vector3d far_pt_1;
    for (int i=0;i<3;i++) far_pt_1[i]=opv.far_pt_1[i];
    Eigen::Vector3d near_pt_2;
    for (int i=0;i<3;i++) near_pt_2[i]=opv.near_pt_2[i];
    Eigen::Vector3d far_pt_2;
    for (int i=0;i<3;i++) far_pt_2[i]=opv.far_pt_2[i];

    Eigen::Vector3d pts1_delta = far_pt_1-near_pt_1;
    Eigen::Vector3d pts2_delta = far_pt_2-near_pt_2;
    int npts1 = std::ceil(pts1_delta.norm()/m_resolution);
    int npts2 = std::ceil(pts2_delta.norm()/m_resolution);
    for (int i=0;i<=npts1;i++) {
      Eigen::Vector3d p1 = near_pt_1+double(i/npts1)*pts1_delta;
      for (int j=0;j<=npts2;j++) {
        Eigen::Vector3d p2 = near_pt_2+double(j/npts2)*pts2_delta;
        //descritize p1->p2 line
        Eigen::Vector3d pts3_delta = p2-p1;
        int npts3 = std::ceil(pts3_delta.norm()/m_resolution);
        for (int k=0;k<=npts3;k++)
        {
          Eigen::Vector3d p = p1+double(k/npts3)*pts3_delta;
          double ix=(p(0)-workspace_lb(0))/m_resolution;
          double iy=(p(1)-workspace_lb(1))/m_resolution;
          double iz=(p(2)-workspace_lb(2))/m_resolution;

          if ( (ix<0) || (ix>=m_npnt[0]))
            continue;
          if ( (iy<0) || (iy>=m_npnt[1]))
            continue;
          if ( (iz<0) || (iz>=m_npnt[2]))
            continue;

          occup(int(std::floor(ix)),
              int(std::floor(iy)),
              int(std::floor(iz)))=opv.prob_successful_passage;
          occup(int(std::floor(ix)),
              int(std::floor(iy)),
              int(std::ceil(iz)))=opv.prob_successful_passage;
          occup(int(std::floor(ix)),
              int(std::ceil(iy)),
              int(std::floor(iz)))=opv.prob_successful_passage;
          occup(int(std::floor(ix)),
              int(std::ceil(iy)),
              int(std::ceil(iz)))=opv.prob_successful_passage;
          occup(int(std::ceil(ix)),
              int(std::floor(iy)),
              int(std::floor(iz)))=opv.prob_successful_passage;
          occup(int(std::ceil(ix)),
              int(std::floor(iy)),
              int(std::ceil(iz)))=opv.prob_successful_passage;
          occup(int(std::ceil(ix)),
              int(std::ceil(iy)),
              int(std::floor(iz)))=opv.prob_successful_passage;
          occup(int(std::ceil(ix)),
              int(std::ceil(iy)),
              int(std::ceil(iz)))=opv.prob_successful_passage;
        }
      }
    }
    m_occupancy=(m_occupancy*occup);
  }
  // m_occupancy = 1/m_occupancy;

}

double ROPMetrics::totalROP(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points)
{
  double total_occupancy=1.0;
  double ix;
  double iy;
  double iz;


  for (const Eigen::Vector3d& p: points)
  {
    ix=(p(0)-m_x_min(0))*m_inv_resolution(0);
    if ( (ix<0) || (ix>=m_npnt))
      continue;

    iy=(p(1)-m_x_min(1))*m_inv_resolution(1);
    if ( (iy<0) || (iy>=m_npnt))
      continue;

    iz=(p(2)-m_x_min(2))*m_inv_resolution(2);
    if ( (iz<0) || (iz>=m_npnt))
      continue;

    double prob_success=            m_occupancy(int(std::floor(ix)),int(std::floor(iy)),int(std::floor(iz) ));
    prob_success=std::min(prob_success,m_occupancy(int(std::floor(ix)),int(std::floor(iy)),int(std::ceil(iz)  )));
    prob_success=std::min(prob_success,m_occupancy(int(std::floor(ix)),int(std::ceil(iy) ),int(std::floor(iz) )));
    prob_success=std::min(prob_success,m_occupancy(int(std::floor(ix)),int(std::ceil(iy) ),int(std::ceil(iz)  )));
    prob_success=std::min(prob_success,m_occupancy(int(std::ceil(ix) ),int(std::floor(iy)),int(std::floor(iz) )));
    prob_success=std::min(prob_success,m_occupancy(int(std::ceil(ix) ),int(std::floor(iy)),int(std::ceil(iz)  )));
    prob_success=std::min(prob_success,m_occupancy(int(std::ceil(ix) ),int(std::ceil(iy) ),int(std::floor(iz) )));
    prob_success=std::min(prob_success,m_occupancy(int(std::ceil(ix) ),int(std::ceil(iy) ),int(std::ceil(iz)  )));
    total_rop*=prob_success;
  }
  // total_occupancy*=m_voxel_volume;
  return 1/total_rop;
}


double ROPMetrics::occupiedROPMultiplier(const Eigen::VectorXd &q)
{
  unsigned ipnt=0;
  m_transformations=chain->getTransformations(q);
  for (std::string link_name:links_to_check) {
    auto il = std::find(link_names.begin(),link_names.end(),link_name);
    if ( il != link_names.end() )
    {
      const Eigen::Affine3d& t= m_transformations.at(il);
      const auto& pnts= m_test_points.at(link_name);
      for (unsigned int ip=0;ip<pnts.size();ip++)
      {
        m_points.at(ipnt++)=t*pnts.at(ip);
      }
    }
  }
  return totalROP(m_points);
}

double ROPMetrics::cost(const NodePtr& parent, const NodePtr& new_node)
{
  return cost(parent->getConfiguration(), new_node->getConfiguration());
}

double ROPMetrics::cost(const Eigen::VectorXd& parent, const Eigen::VectorXd& new_node)
{    
  double length = (new_node - parent).norm();
  if (length < 1e-6)
    return length;

  double cost = 0.0;
  unsigned int nsteps = std::ceil(length / step_);
  double inv_nsteps = 1.0 / nsteps;
  double distance = length / nsteps;
  Eigen::VectorXd diff = (configuration2 - configuration1)* inv_nsteps;
  for (unsigned int istep = 0; istep <= nsteps; istep++)
  {
    Eigen::VectorXd q = configuration1 + diff * istep;
    //check for intersection of voxels
    cost += diff*occupiedROPMultiplier(q);
  }

  return cost;
}

double ROPMetrics::utopia(const Eigen::VectorXd &configuration1, const Eigen::VectorXd &configuration2)
{
  return (inv_max_speed_.cwiseProduct(configuration1 - configuration2)).cwiseAbs().maxCoeff()+nu_*(configuration2-configuration1).norm();
}

}
