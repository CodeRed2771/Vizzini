package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.CurrentBreaker;
import com.coderedrobotics.libs.Logger;
import com.coderedrobotics.libs.PWMController;
import com.coderedrobotics.vizzini.statics.Calibration;
import com.coderedrobotics.vizzini.statics.Wiring;
import java.util.function.Function;

/**
 *
 * @author michael
 */
public class Pickup {
    
    // NEED TO ADD VOLTAGE MONITORING CODE TO AUTO STOP WHEELS
    
    private final CurrentBreaker frontBreaker;
    private final PWMController frontWheels;
    private final PWMController rearWheels;
    private final Function pickupEvent;
    
    private boolean pickingUp = false;
//    private boolean droppingInShooter = false;
    
    public Pickup(int frontWheelPort, int rearWheelPort, Function pickupEvent) {
        frontWheels = new PWMController(frontWheelPort, false); // 1 --> suck in
        rearWheels = new PWMController(rearWheelPort, true);
        frontBreaker = new CurrentBreaker(null, Wiring.PICKUP_FRONT_PDP, 
                Calibration.PICKUP_FRONT_CURRENT_THRESHOLD, 
                Calibration.PICKUP_FRONT_CURRENT_TIMEOUT, 
                Calibration.PICKUP_FRONT_CURRENT_IGNORE_DURATION);
        this.pickupEvent = pickupEvent;
//        rearBreaker = new CurrentBreaker(null, Wiring.PICKUP_REAR_PDP,
//                Calibration.PICKUP_REAR_CURRENT_THRESHOLD, 
//                Calibration.PICKUP_REAR_CURRENT_TIMEOUT, 
//                Calibration.PICKUP_REAR_CURRENT_IGNORE_DURATION);
    }
    
    public void feedIn() {
        frontBreaker.reset();
        frontWheels.set(Calibration.PICKUP_INTAKE_SPEED);
        pickingUp = true;
    }
    
    public void feedOut() {
        frontWheels.set(-Calibration.PICKUP_OUTPUT_SPEED);
    }
    
    public void allStop() {
        frontWheels.set(0);
        rearWheels.set(0);
        pickingUp = false;
//        droppingInShooter = false;
    }
    
    public void dropBallInShooter() {
        rearWheels.set(Calibration.PICKUP_SHOOTER_DROP_SPEED);
//        droppingInShooter = true;
    }
    
    public void stopShooterTriggerWheels() {
        rearWheels.set(0);
    }
    
    public void tick() {
        Logger.getInstance().log(String.valueOf(frontBreaker.getCurrent()));
        if (pickingUp && frontBreaker.step()) {
            frontWheels.set(0);
            pickingUp = false;
            pickupEvent.apply(null);
        }
        
//        if (droppingInShooter && rearBreaker.step()) {
//            rearWheels.set(0);
//            droppingInShooter = false;
//        }
    }
}
