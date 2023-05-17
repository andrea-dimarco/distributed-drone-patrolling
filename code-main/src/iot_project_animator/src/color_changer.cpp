#include <cstdio>
#include <iostream>
#include <string>
#include <memory>
#include <unordered_map>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "iot_project_interfaces/srv/color_target.hpp"
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/quaternion.pb.h>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/visual.pb.h>
#include <ignition/msgs/material.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/MessageTypes.hh>

using namespace std::chrono_literals;


const std::string WORLD_NAME = "iot_project_world";



// Function used to get the tree of entities in the simulation.
// Props to ChatGPT for saving me time with this stuff. Working on strings like this in C++ is a pain.
std::tuple<std::unordered_map<int, std::string>, std::unordered_map<int, int>> getSimulationGraph() {

    ignition::transport::Node gazebo_node;

    using namespace std;

    // With these lines, we use the Gazebo service /scene/graph to get the graph
    // (a tree to be more precise) of all the entities in the simulation.    
    ignition::msgs::Empty req_empty;
    ignition::msgs::StringMsg graphResponse;
    bool result;
    unsigned int timeout_graph = 5000;
    gazebo_node.Request("/world/"+ WORLD_NAME +"/scene/graph", req_empty, timeout_graph, graphResponse, result);
    string graphString;
    graphResponse.SerializeToString(&graphString);

    // We then parse the result of the service to a tree stored with these two variables
    unordered_map<int, string> nodes;
    unordered_map<int, int> parents;

    size_t pos = 0;
    while (pos < graphString.length()) {
        size_t lineEnd = graphString.find_first_of("\n", pos);
        string line = graphString.substr(pos, lineEnd - pos);
        pos = lineEnd + 1;

        // check if the line is a node or an edge
        if (line.find("->") == string::npos) {
            size_t idStart = line.find_first_of("0123456789");

            // With this, we skip unwanted lines in the output
            if (idStart == string::npos)
                continue;

            // NOTE: This thing won't work correctly if instances have parenthesis ()
            // in their name
            size_t idEnd = line.find_first_of(" ", idStart);
            int nodeId = stoi(line.substr(idStart, idEnd - idStart));
            size_t labelStart = line.find("label=");
            size_t labelEnd = line.find("(", labelStart + 7);
            string nodeLabel = line.substr(labelStart + 7, labelEnd - 1 - labelStart - 7);
            nodes[nodeId] = nodeLabel;

        }
        else {
            // it's an edge
            size_t srcStart = line.find_first_of("0123456789");
            size_t srcEnd = line.find_first_of(" ", srcStart);
            int srcNodeId = stoi(line.substr(srcStart, srcEnd - srcStart));
            size_t dstStart = line.find_first_of("0123456789", srcEnd + 1);
            size_t dstEnd = line.find_first_of(" ", dstStart);
            int dstNodeId = stoi(line.substr(dstStart, dstEnd - dstStart));
            parents[dstNodeId] = srcNodeId;
        }
    }

    // And then return both of these in a tuple
    return make_tuple(nodes, parents);
}

// Function used to get an association between the sphere in the simulation and their visual.
// Due to the way target pheres are spawned, we lose the association between their name and 
// their visual component (two levels nested: sphere->link->visual), so it is impossible
// (is it? I'm not able to do it, if you are able to, please tell me how), to reference its
// parent's parent when using the service set_visual to change its color.
std::unordered_map<std::string, int> getVisualAssociation() {

    using namespace std;

    unordered_map<int, string> nodes;
    unordered_map<int, int> parents;
    tie(nodes, parents) = getSimulationGraph();


    unordered_map<string, int> visualAssociation;

    for (auto node : nodes) {
        if (node.second == "sphere_visual")
            visualAssociation[nodes[parents[parents[node.first]]]] = node.first;
    }

    return visualAssociation;

}

// int getVisual(std::unordered_map<std::string, int> association, std::string target) {
//     return association[target];
// }


auto visualAssociation = getVisualAssociation();



void changeTargetColor(const std::shared_ptr<iot_project_interfaces::srv::ColorTarget::Request> request,
                  std::shared_ptr<iot_project_interfaces::srv::ColorTarget::Response> response) {


    using namespace ignition::msgs;

    ignition::transport::Node gazebo_node;
    auto target = visualAssociation[request->target];



    Visual req;
    req.set_type(Visual_Type_ENTITY);
    req.set_id(target);

    Color * new_diffuse = new Color();
    new_diffuse->set_r(request->r);
    new_diffuse->set_g(request->g);
    new_diffuse->set_b(request->b);
    new_diffuse->set_a(request->a);

    Material * new_material = new Material();
    new_material->set_allocated_diffuse(new_diffuse);

    req.set_allocated_material(new_material);

    Boolean rep;
    unsigned int timeout = 5000;
    bool result;

    gazebo_node.Request("/world/"+WORLD_NAME+"/visual_config", req, timeout, rep, result);

    response->result = result;

}


int main(int argc, char ** argv) {
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);

    // for (auto association : visualAssociation)
    // std::cout << association.first << " : " << association.second << std::endl;

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("color_changer");
    rclcpp::Service<iot_project_interfaces::srv::ColorTarget>::SharedPtr service =
        node->create_service<iot_project_interfaces::srv::ColorTarget>("/iot_animator/change_color", &changeTargetColor);

    rclcpp::spin(node);

    rclcpp::shutdown();


    return 0;
}


