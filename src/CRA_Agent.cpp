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

class CRA_Agent: public PatrolAgent {

private:
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


void CRA_Agent::init(int argc, char** argv) {
  
  	PatrolAgent::init(argc,argv);

  	
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
void CRA_Agent::processEvents() {
    
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

int CRA_Agent::compute_next_vertex() {
    return conscientious_reactive_avoidant(current_vertex, vertex_web, instantaneous_idleness, tab_intention, NUMBER_OF_ROBOTS);
}


void CRA_Agent::send_results() {   
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
}

void CRA_Agent::receive_results() {
  
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

    CRA_Agent agent;
    agent.init(argc,argv);    
    agent.run();

    return 0; 
}
