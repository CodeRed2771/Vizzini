package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PWMController;
import com.coderedrobotics.vizzini.statics.Wiring;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 *
 * @author michael
 */
public class Arm {
    
    // will need to add limit switches and an encoder (maybe)
    private final PWMController arm;
    private PIDControllerAIAO pidController;
    private final Pickup pickup;
    private final DigitalInput limitSwitch;
   
    public static final int RED_AND_GREEN_LEDS = 0;
    public static final int BLUE_LEDS = 1;
    
    public Arm(int armMotorPort, int pickupFrontMotorPort, int pickupRearMotorPort) {
        pickup = new Pickup(pickupFrontMotorPort, pickupRearMotorPort);
        arm = new PWMController(armMotorPort, false);
        limitSwitch = new DigitalInput(Wiring.ARM_LIMIT_SWITCH);
    }
    
    public void move(double speed) {
        if (speed > 0 && limitSwitch.get()) {
            speed = 0;
        }
        arm.set(speed); // also need to implement movement with pid controller
    }
    
    public void gotoShootPosition() {
        // Uses PID Controller to move arm to position set in Calibration
    }
    
    public void disablePIDController() {
        // Full manual.  In case of encoder failure.
    }
    
    public void feedIn() {
        pickup.feedIn();
    }
    
    public void feedOut() {
        pickup.feedOut();
    }
    
    public void feedStop() {
        pickup.allStop();
    }
    
    public void dropBallInShooter() {
        pickup.dropBallInShooter();
    }
    
    public void tick() {
        pickup.tick();
    }
}
