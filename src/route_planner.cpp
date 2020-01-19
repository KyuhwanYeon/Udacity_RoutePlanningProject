#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage.
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  // Using the m_Model.FindClosestNode method to find the closest nodes to the
  // starting and ending coordinates. Storing the found nodes in the
  // RoutePlanner's start_node and end_node attributes.
  start_node = &model.FindClosestNode(start_x, start_y);
  end_node = &model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  float h;
  h = node->distance(*end_node);
  return h;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->g_value =
        current_node->g_value + current_node->distance(*neighbor);
    // neighbor g_value = cur_g + distance from cur to neighbor
    // Debug std::cout<<current_node->distance(*neighbor)<<std::endl;
    neighbor->h_value = CalculateHValue(neighbor);
    // negihbor h_value = distance from neighbor to end_node
    neighbor->visited = true;
    // neighbor is visited
    open_list.push_back(neighbor);
    // add neighbor to open_list
  }
}


bool compareFVal(const RouteModel::Node *a, const RouteModel::Node *b) {
  float f1 = (a->g_value + a->h_value);
  float f2 = (b->g_value + b->h_value);
  return f1 > f2;
  // return (a->g_value + a->h_value) > (b->g_value + b->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(open_list.begin(), open_list.end(),
            compareFVal); // Descending order ex: 2.1 2.0 1.9,..
  RouteModel::Node *OptimalNode =
      open_list.back(); // since open_list has decending order now, last node of
                        // open_list has lowest f_value! which is optimal choice
                        // for next node
  // 	std::cout<<"lowest_f_value= "<<lowest_f_val_node->g_value +
  // lowest_f_val_node->g_value<<std::endl;
  open_list.pop_back();
  return OptimalNode;
}

std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  std::vector<RouteModel::Node> path_found;

  while (current_node->x != start_node->x && current_node->y != start_node->y) {
    distance += current_node->distance(
        *(current_node->parent)); // add distance from current to parent in
                                  // private member variable distance
    path_found.push_back(*current_node); // add path_found;
    current_node = current_node->parent; // move current node to parent node
  }
  path_found.push_back(*current_node); // add start_node as last element

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of
                                     // the map to get meters.
  std::reverse(path_found.begin(),
               path_found.end()); // change the order : start_node-> first
                                  // element, end_node -> last element

  return path_found;
}


void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;
  start_node->visited = true;
  open_list.push_back(start_node);

  while (!open_list.empty()) {

    current_node = NextNode();

    if (current_node->distance(*this->end_node) == 0) {
      m_Model.path = ConstructFinalPath(current_node);
      return;
    } else {
      AddNeighbors(current_node);
    }
  }
  // TODO: Implement your solution here.
}