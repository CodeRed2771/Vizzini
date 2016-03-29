package com.coderedrobotics.vizzini;


import com.coderedrobotics.libs.PWMController;
import com.coderedrobotics.vizzini.statics.Calibration;


/**
 *
 * @author michael
 */
public class Lift {
    
    // two motors
    private PWMController tapeMeasure;
    private PWMController lift;

    // contstructor (2 wiring ports)
    public Lift(int tapePort, int liftPort) {
        tapeMeasure = new PWMController(tapePort, false);
        lift = new PWMController(liftPort, false);
    }
    
// move tape measure (double)
    public void tapeMeasure(double speed) {
        tapeMeasure.set(speed);
    }
    
    // lift out
    public void liftOut(){
        lift.set(-Calibration.LIFT_SPEED);
    }
    
    // lift in
    
    public void liftIn(){
        lift.set(Calibration.LIFT_SPEED);
    }
    
    public void stop() {
        lift.set(0);
    }
}
