#include "thruster_interface.hpp"

#include "config.hpp"

Servo g_thrusters[NUMBER_OF_THRUSTERS];
const uint8_t g_kPinMap[NUMBER_OF_THRUSTERS] = THRUSTER_PINS;
const uint32_t g_initPwm[NUMBER_OF_THRUSTERS] = INIT_THRUSTER_PWM;
/**
 * @brief Initialize thrusters, set initial PWM for idle state.
 *
 * Attaches each thruster to its pin, sets PWM to INIT_THRUSTER_PWM (idle
 * state), readies thrusters for control without active propulsion.
 *
 * @note Ensure that g_thrusters[] and g_kPinMap[] are configured.
 *
 * @param None
 * @return None
 */

void initializeThrusters() {
  for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS;
       thruster_index++) {
    g_thrusters[thruster_index].attach(g_kPinMap[thruster_index]);
    g_thrusters[thruster_index].writeMicroseconds(g_initPwm[thruster_index]);
  }
}

/**
 * @brief Set thruster pwm_values based on throttle values.
 *
 * Sets pwm values for each thruster using provided throttle values.
 * Throttle values are scaled from [-100, 100] to [1000, 2000] (PWM range).
 *
 * @param throttles An array of int8_t values representing throttle for each
 * thruster. Array size should be NUMBER_OF_THRUSTERS.
 *
 * @return None
 */

void setThrusterThrottle(const int32_t *pwm_values) {
  int pwm_value;
  for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS;
       thruster_index++) {
    pwm_value = pwm_values[thruster_index];
    g_thrusters[thruster_index].writeMicroseconds(pwm_value);
  }
}
// void initializeDropper(){
//   pinMode(m1,OUTPUT);
//   pinMode(m2,OUTPUT);
//   pinMode(mpwm,OUTPUT);
//   digitalWrite(m1,LOW);
//   digitalWrite(m2,LOW);
//   analogWrite(mpwm,0);
// }

// void activateDropper(){
//     digitalWrite(m1,HIGH);
//     digitalWrite(m2,LOW);
//     analogWrite(mpwm,200);
//     delay(500);
//     digitalWrite(m1,LOW);
//     digitalWrite(m2,LOW);
//     delay(3000);
//     digitalWrite(m1,LOW);
//     digitalWrite(m2,HIGH);
//     analogWrite(mpwm,200);
//     delay(500);
//     digitalWrite(m1,LOW);
//     digitalWrite(m2,LOW);
// }
