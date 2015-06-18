#include "modules/finken_model/finken_model_oscillating.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/electrical.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"
#include "modules/finken_model/finken_model_environment.h"


#include "firmwares/rotorcraft/autopilot.h"
#include <math.h>

//struct system_model_s finken_oscillating_model;

uint16_t finken_oscillating_last_time;

bool finken_oscillating_mode;
float height_oscillating_down;
float height_oscillating_up;
float middle;
float time_oscillating;
float height_changing_rate;
bool go_down;
bool search_neighbor;
bool check_direction;
int state;

enum sonar_direction{front, side};

int16_t getSensorValue(uint8 ac_id, enum sensor_pos){
    
    if( sensor_pos == front ){
        
        return finken_sensor_model.distance_d_front;
        
    } else {
        
        switch (ac_id) {
                
            case 201: //white
                return finken_sensor_model.distance_d_right;
                
            case 202: //purple
                return finken_sensor_model.distance_d_right;
                
            case 203: //green
                return finken_sensor_model.distance_d_left;
                
            case 204: //blue
                return finken_sensor_model.distance_d_left;
                
            default: return 0;
        }
    }
}


void update_actuators_set_point(void);

void finken_oscillating_model_init(void) {

    height_oscillating_down = 0.40;
    height_oscillating_up = 0.9;
    middle = ((height_oscillating_up - height_oscillating_down) / 2) + height_oscillating_down;
    finken_oscillating_last_time = 0;
    height_changing_rate = 0.05;
    go_down = false;
    finken_oscillating_mode = false;
    search_neighbor = true;
    check_direction = false;
    state = 0;
}

void finken_oscillating_model_periodic(void)
{
        if ( finken_oscillating_mode ) {
        
            switch ( state ){
                case 0: // no copters found
                    if ( getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND &&
                         getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND) {
                        search_neighbor = false;
                        check_direction = false;
                        state = 2;
                    } else {
                        if ( getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND ||
                             getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND) {
                            search_neighbor = true;
                            check_direction = true;
                            state = 1;
                        } else {
                            search_neighbor = true;
                            check_direction = false;
                            state = 0;
                        }
                    }
                    break;
                case 1: // one copter found
                    if ( getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND &&
                         getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND) {
                        search_neighbor = false;
                        check_direction = false;
                        state = 2;
                    } else {
                        if ( getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND ||
                             getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND) {
                            search_neighbor = true;
                            check_direction = false;
                            state = 1;
                        } else {
                            search_neighbor = true;
                            check_direction = false;
                            state = 0;
                        }
                    }
                    break;
                case 2: // both copters found
                    if ( getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND &&
                         getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND) {
                        search_neighbor = false;
                        check_direction = false;
                        state = 2;
                    } else {
                        if ( getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND ||
                             getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND) {
                            search_neighbor = true;
                            check_direction = true;
                            state = 1;
                        } else {
                            search_neighbor = true;
                            check_direction = true;
                            state = 0;
                        }
                    }
                    break;
                default: break;
            }
        }

        if ( (finken_oscillating_last_time + 1) <= stage_time ){

            finken_oscillating_last_time = stage_time;
            
            if( search_neighbor ){
                
                // check weather go up or down
                if ( check_direction == true ) {
                    
                    // if we are above middle, go downwards and otherwise
                    if ( finken_sensor_model.distance_z >= middle ) {
                        go_down = true;
                    } else {
                        go_down = false;
                    }
                    check_direction = false;
                }
            
                if ( go_down ){
                    if ( finken_system_set_point.distance_z > height_oscillating_down ){
                        finken_system_set_point.distance_z -= height_changing_rate;
                    } else {
                        finken_system_set_point.distance_z = height_oscillating_down;
                        go_down = false;
                    }
                
                } else {
                    if ( finken_system_set_point.distance_z < height_oscillating_up ){
                        finken_system_set_point.distance_z += height_changing_rate;
                    } else {
                        finken_system_set_point.distance_z = height_oscillating_up;
                        go_down = true;
                    }
                }
            }
        }
    }
}

//void update_finken_oscillating_model(void)
//{
//	if(finken_sensor_model.distance_z < 2.5) {
//		finken_oscillating_model.distance_z     = finken_sensor_model.distance_z;
//	}
//	
//  finken_oscillating_model.velocity_theta = finken_sensor_model.velocity_theta;
//  finken_oscillating_model.velocity_x     = finken_sensor_model.velocity_x;
//  finken_oscillating_model.velocity_y     = finken_sensor_model.velocity_y;
//}
//
//void send_finken_oscillating_model_telemetry(struct transport_tx *trans, struct link_device* link)
//{
//  trans=trans;
//  link=link;
//  DOWNLINK_SEND_FINKEN_SYSTEM_MODEL(
//    DefaultChannel,
//    DefaultDevice,
//    &finken_oscillating_model.distance_z,
//    &finken_oscillating_model.velocity_theta,
//    &finken_oscillating_model.velocity_x,
//    &finken_oscillating_model.velocity_y,
//    &finken_actuators_set_point.alpha,
//    &finken_actuators_set_point.beta,
//    &finken_actuators_set_point.thrust
//  );
//}
