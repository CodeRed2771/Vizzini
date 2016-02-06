package com.coderedrobotics.vizzini;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;

import com.coderedrobotics.libs.PIDControllerAIAO;

import edu.wpi.first.wpilibj.Encoder;

public class Shooter {

    // ADD ENCODER, PID Controller, and Voltage Monitoring
    
//    Talon shooter;
//    PIDControllerAIAO pid;
//    Encoder enc;
//    double spinRate = 0;
//    final double changeRate = .25;
    public Shooter() {
//        shooter = new Talon(0);
//        enc = new Encoder(0, 1);
//        pid = new PIDControllerAIAO(.5, 0, 0, enc, shooter, true, "Shooter");
//        pid.setOutputRange(-.2, .2);
//        enc.setPIDSourceType(PIDSourceType.kRate);
    }

    public boolean isSpunUp() {
        // This method will return true if the encoder reports that the proper
        // speed has been achieved
        return true;
    }

    public void spinUp() {

    }

    public void stop() {
        // Stops the wheels
    }

    public void stopWithDelay() {
        // Stops the wheels after a certain time interval, settable in Calibration
    }

    public boolean hasFired() {
        // Uses voltage regulation to monitor whether a ball has gone through
        // the shooter wheels.
        return true;
    }
}
