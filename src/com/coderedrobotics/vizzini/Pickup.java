package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.CurrentBreaker;
import com.coderedrobotics.libs.PWMController;
import com.coderedrobotics.vizzini.statics.Calibration;
import com.coderedrobotics.vizzini.statics.Wiring;

/**
 *
 * @author michael
 */
public class Pickup {
    
    // NEED TO ADD VOLTAGE MONITORING CODE TO AUTO STOP WHEELS
    
    private final CurrentBreaker frontBreaker;
    private final CurrentBreaker rearBreaker;
    private final PWMController frontWheels;
    private final PWMController rearWheels;
    
    private boolean pickingUp = false;
    private boolean droppingInShooter = false;
    
    public Pickup(int frontWheelPort, int rearWheelPort) {
        frontWheels = new PWMController(frontWheelPort, false); // 1 --> suck in
        rearWheels = new PWMController(rearWheelPort, true);
        frontBreaker = new CurrentBreaker(null, Wiring.PICKUP_FRONT_PDP, 
                Calibration.PICKUP_FRONT_CURRENT_THRESHOLD, Calibration.PICKUP_FRONT_CURRENT_TIMEOUT);
        rearBreaker = new CurrentBreaker(null, Wiring.PICKUP_REAR_PDP,
                Calibration.PICKUP_REAR_CURRENT_THRESHOLD, Calibration.PICKUP_REAR_CURRENT_TIMEOUT);
    }
    
    public void feedIn() {
        frontWheels.set(Calibration.PICKUP_INTAKE_SPEED);
        frontBreaker.reset();
        pickingUp = true;
    }
    
    public void feedOut() {
        frontWheels.set(-Calibration.PICKUP_OUTPUT_SPEED);
    }
    
    public void allStop() {
        frontWheels.set(0);
        rearWheels.set(0);
        pickingUp = false;
        droppingInShooter = false;
    }
    
    public void dropBallInShooter() {
        rearWheels.set(Calibration.PICKUP_SHOOTER_DROP_SPEED);
        rearBreaker.reset();
        droppingInShooter = true;
    }
    
    public void tick() {
        if (pickingUp && frontBreaker.step()) {
            frontWheels.set(0);
            pickingUp = false;
        }
        
        if (droppingInShooter && rearBreaker.step()) {
            rearWheels.set(0);
            droppingInShooter = false;
        }
    }
}