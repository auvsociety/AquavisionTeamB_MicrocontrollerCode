#ifndef MAIN_HPP
#define MAIN_HPP
#include <Arduino.h>

#include "communication_interface.hpp"
#include "config.hpp"
#include "sensor_interface.hpp"
// #include "sensor_math.hpp"
#include "thruster_interface.hpp"
#include "diagnostics.hpp"

#define UPDATE_RATE 50
#define PUBLISH_RATE 10

#define BNO08X_RESET -1    //Beacuse we are using I2C



#endif  // MAIN_HPP