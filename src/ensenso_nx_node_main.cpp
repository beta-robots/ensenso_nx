
//ros dependencies
#include "ensenso_nx_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "ensenso_nx_node");
      
      //create ros wrapper object
      EnsensoNxNode ensenso;
      
      //set node loop rate
      ros::Rate loop_rate(ensenso.rate());
      
      //node loop 
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce(); 
            
            //switch according run mode
            switch(ensenso.runMode())
            {
                case SERVER:
                    //nothing to do, ROS spin will do the job
                    break;
                    
                case PUBLISHER:
                    //just publish the cloud
                    ensenso.publish(); 
                    break;
                    
                default:
                    break;
            }
            
            //relax to fit output rate
            loop_rate.sleep();            
      }
            
      //exit program
      return 0;
}