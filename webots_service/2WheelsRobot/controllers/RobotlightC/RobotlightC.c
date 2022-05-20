/*
 * File:          RobotlightC.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/compass.h>


/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
typedef enum { EMITTER, RECEIVER } robot_types;
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
   WbDeviceTag communication;
  robot_types robot_type;
  int message_printed = 0;
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
if (strncmp(wb_robot_get_name(), "RL", 13) == 0) {
    robot_type = EMITTER;
    communication = wb_robot_get_device("trans1");

    /* if the cannel is not the right one, change it */
    const int channel = wb_emitter_get_channel(communication);
    
  } else if (strncmp(wb_robot_get_name(), "RL1", 14) == 0) {
    robot_type = RECEIVER;
    communication = wb_robot_get_device("rec1");
    wb_receiver_enable(communication, TIME_STEP);
  } else {
    printf("Unrecognized robot name '%s'. Exiting...\n", wb_robot_get_name());
    wb_robot_cleanup();
    return 0;
  }
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     if (robot_type == EMITTER) {
      /* send null-terminated message */
      const char *message = "robot_type";
      wb_emitter_send(communication, message, strlen(message) + 1);
    } else {
      /* is there at least one packet in the receiver's queue ? */
      if (wb_receiver_get_queue_length(communication) > 0) {
        /* read current packet's data */
        const char *buffer = wb_receiver_get_data(communication);

        if (message_printed != 1) {
          /* print null-terminated message */
          printf("Communicating: received \"%s\"\n", buffer);
          message_printed = 1;
        }
        /* fetch next packet */
        wb_receiver_next_packet(communication);
      } else {
        if (message_printed != 2) {
          printf("Communication broken!\n");
          message_printed = 2;
        }
      }
    }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
