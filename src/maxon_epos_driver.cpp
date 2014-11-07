// ros includes
#include <ros/ros.h>
#include <hyraii_msgs/EposCommand.h>
#include <signal.h>
#include <unistd.h>


// asl libraries:
extern "C" {
  #include <libcan/can_serial.h>
  #include <libepos/epos.h>
  #include <libepos/position.h>
  #include <libepos/home.h>
  #include <libepos/error.h>
}

// project includes
#include "../../settings/hyraii-epos.cpp"
#include "../../settings/hyraii-communication.cpp"

//constanst for homing out of the function, so it won't be createt every time
float traveller_pos;

void homing_epos()
{
  ROS_INFO("homing the motors");
  
  // the homing works in the GUI with this settings. it should work here too.
  epos_home(&Foil_1, HY_HOME_TIMEOUT);
  epos_home(&Foil_2, HY_HOME_TIMEOUT);
  epos_home(&Foil_3, HY_HOME_TIMEOUT);
  epos_home(&Rudder, HY_HOME_TIMEOUT);
  
  // test to check the position
  //float pos = epos_get_position(&Foil_1);
  //ROS_INFO("Position Foil 1 [%f]", pos);
  
}

void initialize_epos() 
{ 
  traveller_pos=0;
  ROS_INFO("initialize motors and Epos");
  
  // initialize Can device
  can_init(&dev_Hyraii, &can_Hyraii_config);
    
  // initialize Epos 
  epos_init(&Foil_1, &dev_Hyraii, &epos_Foil_1_config);
  usleep(10000);
  epos_init(&Foil_2, &dev_Hyraii, &epos_Foil_2_config);
  usleep(10000);
  epos_init(&Foil_3, &dev_Hyraii, &epos_Foil_3_config);
  usleep(10000);
  epos_init(&Rudder, &dev_Hyraii, &epos_Rudder_config);
  usleep(100);
  epos_init(&Traveller, &dev_Hyraii, &epos_Traveller_config);
  usleep(100);
  //epos_init(&Unterliek, &dev_Hyraii, &epos_Unterliek_config);
  //usleep(100);
  
  
  //resetting epos
  epos_error_clear_history(&Foil_1.dev);
  usleep(100);
  epos_error_clear_history(&Foil_2.dev);
  usleep(100);
  epos_error_clear_history(&Foil_3.dev);
  usleep(100);
  epos_error_clear_history(&Rudder.dev);
  usleep(100);
  epos_error_clear_history(&Traveller.dev);
  usleep(100);
  //epos_error_clear_history(&Unterliek.dev);
  //usleep(100);
  
  //start all epos on bus
  epos_open(&Foil_1); 
  epos_open(&Foil_2); 
  epos_open(&Foil_3); 
  epos_open(&Rudder); 
  epos_open(&Traveller); 
  //epos_open(&Unterliek); 
  
  //start position mode
  epos_position_t pos;
  epos_position_init(&pos, 0);
  	    
  pos.min_value = -100000;
  pos.max_value = 100000;
  pos.max_error = 100000;
  
  //set PID parameter
  epos_position_setup(&Foil_1, hy_foil1_pos_params);
  epos_position_setup(&Foil_2, hy_foil2_pos_params);
  epos_position_setup(&Foil_3, hy_foil3_pos_params);
  epos_position_setup(&Rudder, hy_rudder_pos_params);
  epos_position_setup(&Traveller, hy_traveller_pos_params);
  //epos_position_setup(&Unterliek, hy_unterliek_pos_params);
   
  epos_position_start(&Foil_1, &pos);
  epos_position_start(&Foil_2, &pos);
  epos_position_start(&Foil_3, &pos);
  epos_position_start(&Rudder, &pos);
  epos_position_start(&Traveller, &pos);
  //epos_position_start(&Unterliek, &pos);
  
  // homing epos
  //homing_epos();
}



void eposCallback(const hyraii_msgs::EposCommandConstPtr& cmd)
{
  ROS_INFO("Epos command received");

  
  switch(cmd->type){
  
  //single motor position control
  case SW_MOTOR_CONTROL: 
    for(uint i = 0; i < cmd->motor_id.size(); i++){

	    float target_value = cmd->position[i]*EPOS_FACTOR*EPOS_DIRECTION[cmd->motor_id[i]-1];
	    if(cmd->motor_id[i] == 5){
		traveller_pos += cmd->position[i]*EPOS_FACTOR*EPOS_DIRECTION[cmd->motor_id[i]-1];
		target_value = traveller_pos;
	    }
	    ROS_INFO("try to set motor %i to %f deg",cmd->motor_id[i],target_value);

	    if(cmd->motor_id[i] != 5 && target_value>EPOS_MAX){
	    	target_value = EPOS_MAX;
	    }
	    if(cmd->motor_id[i] != 5 && target_value<EPOS_MIN){
	    	    	target_value = EPOS_MIN;
	    }
	    
	    epos_position_t pos; 
	    epos_position_init(&pos, target_value);
      
	switch((int)cmd->motor_id[i]){
		case 1: 
			if (epos_position_update(&Foil_1, &pos)) {
				ROS_ERROR("epos communication error motor_id:[%i]",cmd->motor_id[i]);
          	}
      		break;  
      case 2: 
	        if (epos_position_update(&Foil_2, &pos)) {
            	ROS_ERROR("epos communication error motor_id:[%i]",cmd->motor_id[i]);
			}
      break;
      case 3: 
	        if (epos_position_update(&Foil_3, &pos)) {
            	ROS_ERROR("epos communication error motor_id:[%i]",cmd->motor_id[i]);
			}
      break;
      case 4: 
	        if (epos_position_update(&Rudder, &pos)) {
            	ROS_ERROR("epos communication error motor_id:[%i]",cmd->motor_id[i]);
			}
      break;
      case 5: 
			if (epos_position_update(&Traveller, &pos)) {
				ROS_ERROR("epos communication error motor_id:[%i]",cmd->motor_id[i]);
			}
      break;
//      case 6: 
//	        if (epos_position_update(&Unterliek, &pos)) {
//            	ROS_ERROR("epos comunication error motor_id:[%i]",cmd->motor_id[i]);
//			}
//      break;
      } 
    }

  break;

  //three motor control
  case SW_THREE_MOTOR_CONTROL: 
  break;
  
  // homing all motors
  case SW_HOMING_MOTORS: 
  homing_epos();
  break;
  
  // reinitialize all motors
  case SW_INITIALIZE_ALL: 
  initialize_epos(); 
  
  break;
  
  }
}

void epos_signaled(int signal) {
  ROS_ERROR("EPOS error signaled.");
}

int main(int argc, char** argv)
{
  signal(SIGINT, epos_signaled);
  
// EPOS init
  initialize_epos();
  
// ROS init
  ros::init(argc, argv, "epos");
  ros::NodeHandle n;
  ros::Subscriber epos_sub = n.subscribe("epos_command", 1, eposCallback);
  ros::spin();

// Disable all Epos  
  epos_close(&Foil_1);
  epos_close(&Foil_2);
  epos_close(&Foil_3);
  epos_close(&Rudder);
  epos_close(&Traveller);
//  epos_close(&Unterliek);
  
  epos_destroy(&Foil_1);
  epos_destroy(&Foil_2);
  epos_destroy(&Foil_3);
  epos_destroy(&Rudder);
  epos_destroy(&Traveller);
//  epos_destroy(&Unterliek);
}

