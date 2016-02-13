package com.coderedrobotics.vizzini.statics;

/**
 *
 * @author michael
 */
public class Wiring {

    // PWM MOTOR CONTROLLERS
    public static final int LEFT_DRIVE_MOTOR1 = 0;
    public static final int LEFT_DRIVE_MOTOR2 = 1;
    public static final int RIGHT_DRIVE_MOTOR1 = 2;
    public static final int RIGHT_DRIVE_MOTOR2 = 3;
    
    public static final int PICKUP_FRONT_MOTOR = 5;
    public static final int PICKUP_REAR_MOTOR = 6;
    public static final int SHOOTER_MOTOR_2 = 7;
    
    // MOTOR CONTROLLER PDP PORTS
    public static final int PICKUP_FRONT_PDP = 14;
    public static final int PICKUP_REAR_PDP = 13;
    public static final int SHOOTER_PDP = 0;

    // CAN MOTOR CONTROLLERS
    public static final int ARM_MOTOR = 1;  
    public static final int SHOOTER_MOTOR_1 = 2;
    
    // DIGITAL INPUT
    public static final int ARM_LIMIT_SWITCH = 0;
    public static final int LEFT_ENCODER_A = 3;
    public static final int LEFT_ENCODER_B = 4;
    public static final int RIGHT_ENCODER_A = 1;
    public static final int RIGHT_ENCODER_B = 2;

    // ANALOG INPUT
    
    // Relays
    public static final int RED_AND_GREEN_LEDS = 0;
    public static final int BLUE_LEDS = 1;
}
