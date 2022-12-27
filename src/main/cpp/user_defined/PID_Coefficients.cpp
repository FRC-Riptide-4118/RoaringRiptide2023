#include "user_defined/PID_Coefficients.h"

// Simple constructor just initializes all values
PID_Coefficients::PID_Coefficients(double kF, double kP, double kI, double kD) {

    this->kF = kF;
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;

}