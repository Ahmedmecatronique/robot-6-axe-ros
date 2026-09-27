/* Tâche micro-ROS : nœud « stm32_micro_ros » sur UART (agent côté Raspberry Pi).
 *
 *  Publie   : /joint_states  (sensor_msgs/JointState, best effort, 100 Hz)  positions RÉELLES (encodeurs)
 *             /stm32/status  (std_msgs/Int32MultiArray, best effort, 10 Hz)
 *  S'abonne : /joint_commands (sensor_msgs/JointState, reliable)  consigne + vitesse de préaction
 *             /emergency_stop (std_msgs/Bool, reliable)            arrêt + heartbeat 10 Hz
 *             /stm32/command  (std_msgs/Int32, reliable)           ENABLE / DISABLE / HOME / ZERO_HERE
 *             /gripper_command (std_msgs/Float32, reliable)        0 = fermée .. 1 = ouverte
 */
#ifndef MICROROS_APP_H
#define MICROROS_APP_H

void microros_task(void *arg);

#endif /* MICROROS_APP_H */
