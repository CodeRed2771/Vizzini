package com.coderedrobotics.vizzini.statics;

/**
 *
 * @author michael
 */
public class Calibration {
    
    public static double PICKUP_INTAKE_SPEED = .75;
    public static double PICKUP_OUTPUT_SPEED  = 1;
    public static double PICKUP_SHOOTER_DROP_SPEED = 1;
     
    public static double PICKUP_FRONT_CURRENT_THRESHOLD = 6;
    public static int PICKUP_FRONT_CURRENT_TIMEOUT = 0;
    public static int PICKUP_FRONT_CURRENT_IGNORE_DURATION = 1000; // in ms
    public static double PICKUP_REAR_CURRENT_THRESHOLD = 2;
    public static int PICKUP_REAR_CURRENT_TIMEOUT = 200;
    public static int PICKUP_REAR_CURRENT_IGNORE_DURATION = 1000; 
    
    public static double ARM_SETPOINT_INCREMENT = .0007;
    public static double ARM_CALIBRATION_MOTOR_SPEED = .3;
    public static double ARM_P = 8;
    public static double ARM_I = 0;
    public static double ARM_D = 10;
    public static double ARM_F = 0;
    
    public static double DRIVE_TRAIN_REDUCTION_FACTOR = 0.5;
    
    public static double SHOOTER_ERROR_TOLERANCE = 1000; // is sped up
    public static double SHOOTER_SPIN_SPEED = -32000;
    public static long SHOOTER_STOP_TIMEOUT = 1000;
    public static double SHOOTER_CURRENT_THRESHOLD = 10;
    public static double SHOOTER_BALL_ERROR_THRESHOLD = 6000; // error required to autostop
    public static double SHOOTER_AUTOSTOP_DELAY = 500;
    public static double SHOOTER_P = -0.0001;
    public static double SHOOTER_I = -.75;
    public static double SHOOTER_D = -0.001;
    public static double SHOOTER_F = -1d / 50000d;
}
