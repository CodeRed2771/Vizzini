package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.CurrentBreaker;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PWMController;
import com.coderedrobotics.vizzini.statics.Calibration;
import com.coderedrobotics.vizzini.statics.Wiring;
import edu.wpi.first.wpilibj.CANTalon;


public class Shooter {

    private CANTalon shooter1;
    private PWMController shooter2;
    private PIDControllerAIAO pid;
    private CurrentBreaker breaker;
    private boolean stopping;
    private long timeout;
    
    public Shooter(int talon, int victor) {
        shooter1 = new CANTalon(talon);
        shooter2 = new PWMController(victor, false);
        pid = new PIDControllerAIAO(0, 0, 0, 0, shooter1, shooter1, true, "Shooter");
        breaker = new CurrentBreaker(null, Wiring.SHOOTER_PDP, 7, 0);
//        shooter = new Talon(0);
//        enc = new Encoder(0, 1);
//        pid = new PIDControllerAIAO(.5, 0, 0, enc, shooter, true, "Shooter");
//        pid.setOutputRange(-.2, .2);
//        enc.setPIDSourceType(PIDSourceType.kRate);
    }

    public boolean isSpunUp() {
        return pid.getError() < Calibration.SHOOTER_ERROR_TOLERANCE;
    }

    public void spinUp() {
        pid.setSetpoint(Calibration.SHOOTER_SPIN_SPEED);
    }

    public void stop() {
        pid.setSetpoint(0);
    }

    public void stopWithDelay() {
        stopping = true;
        timeout = System.currentTimeMillis() + Calibration.SHOOTER_STOP_TIMEOUT;
    }

    public boolean hasFired() {
        return breaker.step();
    }
    
    public void tick() {
        if (stopping && System.currentTimeMillis() > timeout) {
            pid.setSetpoint(0);
            stopping = false;
        }
    }
}
