package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PWMController;

/**
 *
 * @author michael
 */
public class Arm {
    
    // will need to add limit switches and an encoder (maybe)
    private final PWMController arm;
    private PIDControllerAIAO pidController;
    private final Pickup pickup;
    
    public Arm(int armMotorPort, int pickupFrontMotorPort, int pickupRearMotorPort) {
        pickup = new Pickup(pickupFrontMotorPort, pickupRearMotorPort);
        arm = new PWMController(armMotorPort, false);
    }
    
    public void move(double speed) {
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
        pickup.feedStop();
    }
    
    public void dropBallInShooter() {
        pickup.dropBallInShooter();
    }
}
