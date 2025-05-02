#include <sstream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include "PatrolAgent.h"
#include "getgraph.h"
#include "algorithms.h"

#include <limits>
#include <vector>

using namespace std;

// Alias for readability
using PathEntry = tuple<int, double>; // (node, time)
using Path = vector<PathEntry>;

class RHARM_Agent: public PatrolAgent {

private:

  // Scenario parameters
  vector<vector<double>> node_node_distances;
  vector<vector<double>> adjacency_matrix;
  int n_agents;
  // Message passing
  int robot_arrived;
  int vertex_arrived;
  // bool message_received;
  bool arrived;
  bool intention;
  // Path intentions
  Path self_path_intention;
  vector<Path> received_path_intentions;
  // RH-ARM parameters
  double horizon_length;
  
      
public:
    virtual void init(int argc, char** argv);
    virtual int compute_next_vertex();
    virtual void processEvents();
    virtual void send_results();
    virtual void receive_results();    
};


void RHARM_Agent::init(int argc, char** argv) {
  
  	PatrolAgent::init(argc,argv);

    // Construct weighted adjacency matrix from vertex_web object
    
    adjacency_matrix.resize(dimension, vector<double>(dimension));
    node_node_distances.resize(dimension, vector<double>(dimension));
  		
  	for (int i=0; i<dimension; i++) {
  		for (int j=0; j<dimension; j++) {
  			adjacency_matrix[i][j] = 0;
  		}
  		for (int j=0; j<vertex_web[i].num_neigh; j++) {
  			adjacency_matrix[i][vertex_web[i].id_neigh[j]] = vertex_web[i].cost[j] * RESOLUTION;
  		}
  	}
    for (int i = 0; i < dimension; i++) {
        for (int j = 0; j < dimension; j++) {
            cout << adjacency_matrix[i][j] << ' ';
        }
        cout << "\n";
    }

  	// ~~~~ Floyd-Warshall to generate all node-to-node distances ~~~~
  	
  	// Initialise weights
  	for (int i=0; i<dimension; i++) {
  		for (int j=0; j<dimension; j++) {
  			if (adjacency_matrix[i][j] != 0) {
  				node_node_distances[i][j] = adjacency_matrix[i][j];
  			} else if (i ==j) {
  				node_node_distances[i][j] = 0;
  			} else {
  				node_node_distances[i][j] = numeric_limits<double>::max();
  			}
  		}
  	}
  	// Calculate shortest paths
  	for (int k=0; k<dimension; ++k) {
        for (int i=0; i<dimension; ++i) {
            for (int j=0; j<dimension; ++j) {
                if (node_node_distances[i][k] + node_node_distances[k][j] < node_node_distances[i][j]) {
                    // If path through vertex k is shorter, update node_node_distances
                    node_node_distances[i][j] = node_node_distances[i][k] + node_node_distances[k][j];
                }
            }
        }
    }

  	n_agents = atoi(argv[3]);
  	// message_received=false;
    arrived = false;
    horizon_length = 40.0;
  	// Initialise received intentions:
	received_path_intentions.resize(n_agents);
}

// Executed at any cycle when goal is not reached
void RHARM_Agent::processEvents() {
    
    if (arrived && n_agents>1){


        //Update Idleness Table:
        double now = ros::Time::now().toSec();
                
        for(int i=0; i<dimension; i++){
            if (i == vertex_arrived){
                last_visit[vertex_arrived] = now;   
            }           
            instantaneous_idleness[i] = now - last_visit[i];      
        }     
        
        arrived = false;
    }
}

int RHARM_Agent::compute_next_vertex() {

    // return 0;

    // for(int i=0; i<dimension; i++){
    //     cout << instantaneous_idleness[i] << ' ';
    // }
    cout << "\n";
    self_path_intention = rh_arm(ros::Time::now().toSec(), current_vertex, horizon_length, instantaneous_idleness, adjacency_matrix, node_node_distances, self_path_intention, received_path_intentions);
    int next_vertex;
    if (self_path_intention.size() > 1) {
        next_vertex = std::get<0>(self_path_intention[1]);
    } else {
        next_vertex = (current_vertex + 1) % dimension;
    } 
    ROS_INFO("Sending goal - Vertex %d\n", next_vertex);
    // ROS_INFO("Also - %f\n", std::get<1>(self_path_intention[1]));
    // cout << "test\n";
    return next_vertex;
}


void RHARM_Agent::send_results() {   
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    // [ID_ROBOT, msg_type, current_vertex, intended_path[(node 0, time 0)...(node n, time n)]...]
    // Intention vectors are unravelled to allow for messaging over a vector
    std_msgs::Int16MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(RHARM_MSG_TYPE);
    msg.data.push_back(current_vertex);
    // msg.data.push_back(self_path_intention);   
    for (const auto& entry: self_path_intention) {
        int node;
        double time;
        tie(node, time) = entry;
        msg.data.push_back(node);
        msg.data.push_back(static_cast<int>(10.0*time)); // Preserve prescision to 10ths of a second
        cout << node << ' ';
        cout << time << ' ';
    }
    cout << "\n";
    do_send_message(msg);
}

void RHARM_Agent::receive_results() {
  
    // [ID_ROBOT, msg_type, current_vertex, intended_path[(node 0, time 0)...(node n, time n)]]
      
    robot_arrived = vresults[0];
    
    int self_id = ID_ROBOT;
    if (self_id==-1){self_id=0;}
    
  	if ((robot_arrived==self_id) || (vresults[1]!=RHARM_MSG_TYPE)) 
    	return;
    	
    vertex_arrived = vresults[2];
    // Re-ravel path intentions
    Path path;
    for (size_t i = 3; i+1 < vresults.size(); i+= 2) {
        int node = vresults[i];
        double time = static_cast<double>(vresults[i+1]) / 10.0;
        path.emplace_back(node, time);
    }

    received_path_intentions[robot_arrived] = path;
    arrived = true;
    

    //message_received = true;
}

int main(int argc, char** argv) {

    RHARM_Agent agent;
    agent.init(argc,argv);    
    agent.run();

    return 0; 
}
