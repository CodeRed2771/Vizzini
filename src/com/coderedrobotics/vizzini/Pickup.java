package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PWMController;
import com.coderedrobotics.vizzini.statics.Calibration;

/**
 *
 * @author michael
 */
public class Pickup {
    
    // NEED TO ADD VOLTAGE MONITORING CODE TO AUTO STOP WHEELS
    
    private final PWMController frontWheels;
    private final PWMController rearWheels;
    
    public Pickup(int frontWheelPort, int rearWheelPort) {
        frontWheels = new PWMController(frontWheelPort, false); // 1 --> suck in
        rearWheels = new PWMController(rearWheelPort, true); 
    }
    
    public void feedIn() {
        frontWheels.set(Calibration.PICKUP_INTAKE_SPEED);
    }
    
    public void feedOut() {
        frontWheels.set(-Calibration.PICKUP_OUTPUT_SPEED);
    }
    
    public void feedStop() {
        frontWheels.set(0);
        rearWheels.set(0);        
    }
    
    public void dropBallInShooter() {
        rearWheels.set(Calibration.PICKUP_SHOOTER_DROP_SPEED);
    }
}
