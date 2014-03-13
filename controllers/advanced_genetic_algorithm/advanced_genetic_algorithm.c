// File:          advanced_genetic_algorithm.c
// Description:   Robot execution code for genetic algorithm
// Project:       Advanced exercises in Cyberbotics' Robot Curriculum
// Author:        Yvan Bourquin - www.cyberbotics.com
// Date:          January 6, 2010

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/distance_sensor.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#define NUM_SENSORS 11
#define NUM_WHEELS 2
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_WHEELS)

// sensor to wheels multiplication matrix
// each each sensor has a weight for each wheel
double matrix[NUM_SENSORS][NUM_WHEELS];

// 3 IR ground color sensors
#define NB_GROUND_SENS 3
#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2

#define NB_DIST_SENS 8

unsigned short gs_value[NB_GROUND_SENS]={0,0,0};

WbDeviceTag sensors[NUM_SENSORS];  // proximity sensors
WbDeviceTag receiver;              // for receiving genes from Supervisor
WbDeviceTag robot_emitter;
WbDeviceTag robot;

double f[1];
int mili = 0;
double cur_x, prev_x, cur_y, prev_y;

// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
void check_for_new_genes() {
  if (wb_receiver_get_queue_length(receiver) > 0) {
  
    f[0] = 0;
    // check that the number of genes received match what is expected
    assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double));
    
    // copy new genes directly in the sensor/actuator matrix
    // we don't use any specific mapping nor left/right symmetry
    // it's the GA's responsability to find a functional mapping
    memcpy(matrix, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(double));
    
    // prepare for receiving next genes packet
    wb_receiver_next_packet(receiver);
  }
}

static double clip_value(double value, double min_max) {
  if (value > min_max)
    return min_max;
  else if (value < -min_max)
    return -min_max;

  return value;
}

void sense_compute_and_actuate() {
  // read sensor values
  double sensor_values[NUM_SENSORS];
  int i, j;
  for (i = 0; i < NUM_SENSORS; i++)
    sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
    
  // Report results to supervisor
  for(i = NB_DIST_SENS; i < NUM_SENSORS; i ++) gs_value[i - NB_DIST_SENS] = sensor_values[i];
  
  
  if(gs_value[1] < 400) f[0] ++;
  else f[0] -= 1;
  if(gs_value[0] < 400) f[0] ++;
  else f[0] -= 1;
  if(gs_value[2] < 400) f[0] ++;
  else f[0] -= 1;
  
  if(gs_value[1] > 400 && gs_value[1] < 900) f[0] -= 100;
  
  
  for(i=0; i < NB_DIST_SENS; i ++) {
    if(sensor_values[i] > 2000) f[0] -= 1;
  }
  mili ++;
  if( mili > 1000){
    mili = 0;
    
    prev_y = 
    
    
    if(abs(prev_x - cur_x) < 0.01 && abs(prev_y - cur_y) < 0.01) f[0] -= 1000;
  }
  
  
  wb_emitter_send(robot_emitter, f, sizeof(double));

  // compute actuation using Braitenberg's algorithm:
  // The speed of each wheel is computed by summing the value
  // of each sensor multiplied by the corresponding weight of the matrix.
  // By chance, in this case, this works without any scaling of the sensor values nor of the
  // wheels speed but this type of scaling may be necessary with a different problem
  double wheel_speed[NUM_WHEELS] = { 0.0, 0.0 };
  for (i = 0; i < NUM_WHEELS; i++)
    for (j = 0; j < NUM_SENSORS; j++)
      wheel_speed[i] += matrix[j][i] * sensor_values[j];
      
  // clip to e-puck max speed values to avoid warning
  wheel_speed[0] = clip_value(wheel_speed[0], 1000.0);
  wheel_speed[1] = clip_value(wheel_speed[1], 1000.0);

  // actuate e-puck wheels
  wb_differential_wheels_set_speed(wheel_speed[0], wheel_speed[1]);
}

int main(int argc, const char *argv[]) {

  f[0] = 0;
  
  wb_robot_init();  // initialize Webots

  // find simulation step in milliseconds (WorldInfo.basicTimeStep)
  int time_step = wb_robot_get_basic_time_step();
    
  // find and enable proximity sensors
  char name[32];
  int i;
  for (i = 0; i < NB_DIST_SENS; i++) {
    sprintf(name, "ps%d", i);
    sensors[i] = wb_robot_get_device(name); /* proximity sensors */
    wb_distance_sensor_enable(sensors[i],time_step);
  }
  for (; i < NUM_SENSORS; i++) {
    sprintf(name, "gs%d", i-NB_DIST_SENS);
    sensors[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(sensors[i],time_step);
  }
    
  // find and enable receiver
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  
  robot = wb_robot_get_device("EPUCK");
  
  
  
  //find and enable robot_emitter
  robot_emitter = wb_robot_get_device("emitter");
  
  // initialize matrix to zero, hence the robot 
  // wheels will initially be stopped
  memset(matrix, 0.0, sizeof(matrix));
  
  // run until simulation is restarted
  while (wb_robot_step(time_step) != -1) {
    check_for_new_genes();
    sense_compute_and_actuate();
  }
  
  wb_robot_cleanup();  // cleanup Webots
  return 0;            // ignored
}
