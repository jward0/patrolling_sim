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

class MNS_Agent: public PatrolAgent {

private:

  int last_node;
  int last_last_node;
  vector<vector<double>> node_node_distances;
  vector<vector<double>> adjacency_matrix;
  int NUMBER_OF_ROBOTS;
  int *tab_intention;
  bool arrived;
  uint vertex_arrived;
  int robot_arrived;
  bool intention;
  uint vertex_intention;
  int robot_intention;   
      
public:
    virtual void init(int argc, char** argv);
    virtual int compute_next_vertex();
    virtual void processEvents();
    virtual void send_results();
    virtual void receive_results();    
};


void MNS_Agent::init(int argc, char** argv) {
  
  	PatrolAgent::init(argc,argv);
  
  	last_node = -1;
  	last_last_node = -1;
  
    // Construct weighted adjacency matrix from vertex_web object
    
    adjacency_matrix.resize(dimension, vector<double>(dimension));
    node_node_distances.resize(dimension, vector<double>(dimension));
  		
  	for (int i=0; i<dimension; i++) {
  		for (int j=0; j<dimension; j++) {
  			adjacency_matrix[i][j] = 0;
  		}
  		for (int j=0; j<vertex_web[i].num_neigh; j++) {
  			adjacency_matrix[i][vertex_web[i].id_neigh[j]] = vertex_web[i].cost[j];
  		}
  	}
  	
  	// Mean normalise weighted adjacency matrix
  	double sum = 0;
  	double count = 0;
  
  	for (int i=0; i<dimension; i++) {
  		for (int j=0; j<dimension; j++) {
			if (adjacency_matrix[i][j] != 0) {
				sum += adjacency_matrix[i][j];
				count++;
			}
		}
  	}
  	
  	double average = sum / count;
  	
  	for (int i=0; i<dimension; i++) {
  		for (int j=0; j<dimension; j++) {
  			adjacency_matrix[i][j] /= average;
  		}
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
    // Normalise by max node-node distance
    // double max_distance = max_element(begin(node_node_distances[current_vertex]), end(node_node_distances[current_vertex]));
    double max_distance = 0.0;

    for (int i=0; i<dimension; i++) {
        if (node_node_distances[current_vertex][i] > max_distance) {
            max_distance = node_node_distances[current_vertex][i];
        }
    }
    
  	for (int i=0; i<dimension; i++) {
  		for (int j=0; j<dimension; j++) {
  			node_node_distances[i][j] /= max_distance;
  		}
  	}
  	
  	NUMBER_OF_ROBOTS = atoi(argv[3]);
  	arrived=false;
  	intention=false;
    
  	//INITIALIZE tab_intention:
  	tab_intention = new int[NUMBER_OF_ROBOTS];
  	for (int i=0; i<NUMBER_OF_ROBOTS; i++){
    	tab_intention[i] = -1;
  	}

}

// Executed at any cycle when goal is not reached
void MNS_Agent::processEvents() {
    
    if (arrived && NUMBER_OF_ROBOTS>1){


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

    if (intention && NUMBER_OF_ROBOTS>1) {    
        tab_intention[robot_intention] = vertex_intention;
        intention = false;
    }
}

int MNS_Agent::compute_next_vertex() {
    return spatial_priority_network(current_vertex, vertex_web, instantaneous_idleness, tab_intention, NUMBER_OF_ROBOTS, dimension, node_node_distances, last_node, last_last_node);
}


void MNS_Agent::send_results() {   
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    // [ID,msg_type,vertex,intention]
    std_msgs::Int16MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(SEBS_MSG_TYPE);
    msg.data.push_back(current_vertex);
    msg.data.push_back(next_vertex);    
    do_send_message(msg);
    last_last_node = last_node;
    last_node = current_vertex;
}

void MNS_Agent::receive_results() {
  
    std::vector<int>::const_iterator it = vresults.begin();
    int id_sender = *it; it++;
    int msg_type = *it; it++;
    
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    
  	if ((id_sender==value) || (msg_type!=SEBS_MSG_TYPE)) 
    	return;
        
    robot_arrived = vresults[0];
    vertex_arrived = vresults[2];
    arrived = true;
    robot_intention = vresults[0];
    vertex_intention = vresults[3];
    intention = true;
}

int main(int argc, char** argv) {

    MNS_Agent agent;
    agent.init(argc,argv);    
    agent.run();

    return 0; 
}
