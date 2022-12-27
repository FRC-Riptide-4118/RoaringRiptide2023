#pragma once

// This structure will allow for easy management of PID coefficients to avoid random extraneous variables
struct PID_Coefficients {

    // Constructor to initialize all values
    PID_Coefficients(double kF, double kP, double kI, double kD);
    // All values (which will default to public - this is fine since they may be modified outside of the struct)
    double kF;
    double kP;
    double kI;
    double kD;

};