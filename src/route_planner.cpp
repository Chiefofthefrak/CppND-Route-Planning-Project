#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    
    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);
}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    
    return node->distance(*end_node);
}



void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for(auto* neighbourNode: current_node->neighbors){
    
    neighbourNode->parent = current_node;
    
    neighbourNode->h_value = CalculateHValue(neighbourNode);
    
    neighbourNode->g_value = current_node->g_value + current_node->distance(*neighbourNode);
    neighbourNode->visited = true;
    open_list.push_back(neighbourNode);

    
  }
}



bool sumComparison(RouteModel::Node *NodeA, RouteModel::Node *NodeB){
  return NodeA->g_value + NodeA-> h_value > NodeB->g_value +NodeB-> h_value;
}

RouteModel::Node *RoutePlanner::NextNode() {
  
  std::sort(this->open_list.begin(),this->open_list.end(),sumComparison);
  RouteModel::Node* lowestSumNode = this->open_list.back();
  
  this->open_list.pop_back();
  return lowestSumNode;
}




std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

  
    while (current_node->parent != nullptr){
      path_found.push_back(*current_node);
      distance = distance + current_node->distance(*(current_node->parent));
      current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(),path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}



void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    
    this->start_node->visited = true;

    this->open_list.push_back(start_node);
    while(!open_list.empty()){
      current_node = this->NextNode();
      if(current_node==end_node){
        m_Model.path = ConstructFinalPath(current_node);
        return;
      }else{
        AddNeighbors(current_node);
      }

    }

}