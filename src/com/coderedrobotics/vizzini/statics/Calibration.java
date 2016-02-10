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
    
    public static double DRIVE_TRAIN_REDUCTION_FACTOR = 0.5;
}
