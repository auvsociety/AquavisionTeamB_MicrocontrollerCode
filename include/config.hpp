#ifndef CONFIG_HPP
#define CONFIG_HPP
#define NUMBER_OF_THRUSTERS 7
#define INIT_THRUSTER_PWM \
  { 1500, 1500 ,1500 ,1500 ,1500, 1500, 1500}
#define THRUSTER_PINS \
  { PA0, PA1, PA2, PA3, PA6, PA7, PA10}
#define THRUSTER_MIN_PWM 1200 - (1500 - 1472)
#define THRUSTER_MAX_PWM 1800 - (1500 - 1472)
#define NORMAL_MODE 0
#define CALIBRATION_MODE 0
#define CALIBRATION_UPDATE_MODE 2
#define G 9.80665
// const uint8_t m1 = PA4;
// const uint8_t m2 = PA5;
// const uint8_t mpwm = PA7;
#endif  // CONFIG_HPP