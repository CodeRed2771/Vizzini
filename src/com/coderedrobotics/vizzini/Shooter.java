package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PWMController;
import com.coderedrobotics.vizzini.statics.Calibration;
import com.coderedrobotics.vizzini.statics.Wiring;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {

    private final CANTalon shooter1;
    private final PWMController shooter2;
    private final PIDControllerAIAO pid;
    private final AutoStop autoStop;
    private final Relay light;
    private final Servo shooterGate;
    private boolean stopping;
    private double smoothedOutput = 0;
    private long timeout;
    private boolean hasBeenSpunUp = false;
    private double speed = Calibration.SHOOTER_SPIN_SPEED;

    public Shooter(int talon, int victor, int light) {
        shooter1 = new CANTalon(talon);
        shooter2 = new PWMController(victor, false);
        shooterGate = new Servo(Wiring.SHOOTER_GATE_SERVO);
       
        
        pid = new PIDControllerAIAO(Calibration.SHOOTER_P, Calibration.SHOOTER_I,
                Calibration.SHOOTER_D, Calibration.SHOOTER_F, new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }

            @Override
            public double pidGet() {
                return shooter1.getEncVelocity();
            }
        }, (double output) -> {
                    smoothedOutput = ((output * 0.2) + (smoothedOutput * 0.8));
                    shooter1.set(smoothedOutput);
                    shooter2.set(smoothedOutput);
                }, true, "Shooter");
        pid.setOutputRange(0, 1);
        autoStop = new AutoStop();
        this.light = new Relay(light);
        closeGate();
    }

    public void openGate() {
    	shooterGate.setPosition(0.8);
    }
    public void closeGate() {
    	shooterGate.setPosition(.2);
    }
    public boolean isSpunUp() {
        boolean spunUp = Math.abs(pid.getError()) < Calibration.SHOOTER_ERROR_TOLERANCE;
        hasBeenSpunUp = spunUp || hasBeenSpunUp;
        return spunUp;
    }

    public boolean hasBeenSpunUp() {
        isSpunUp(); // update hasBeenSpunUp
        return hasBeenSpunUp;
    }

    public void spinUp() {
        pid.setSetpoint(speed);
        pid.enable();
    }

    public void stop() {
        smoothedOutput = 0;
        pid.setSetpoint(0);
        pid.disable();
        autoStop.reset();
        hasBeenSpunUp = false;
    }

    public void stopWithDelay() {
        stopping = true;
        timeout = System.currentTimeMillis() + Calibration.SHOOTER_STOP_TIMEOUT;
    }

    public boolean hasFired() {
        return autoStop.tick();
    }

    public void tick() {
//        Logger.getInstance().log(String.valueOf(shooter1.getEncVelocity()));
        if (stopping && System.currentTimeMillis() > timeout) {
            stop();
            stopping = false;
        }
        SmartDashboard.putNumber("Servo", shooterGate.getAngle());
    }

    public void enableOverrideMode() {

    }
    
    public void lightOn() {
        light.set(Relay.Value.kOn);
    }

    public void lightOff() {
        light.set(Relay.Value.kOff);
    }
    
    public void toggleLight() {
        light.set(light.get() == Relay.Value.kOn ? Relay.Value.kOff : Relay.Value.kOn);
    }
    
    public void setSpeedStraightOuterWorks() {
        speed = Calibration.SHOOTER_SPIN_SPEED_STRAIGHT;
    }
    
    public void setSpeedLowBar() {
        speed = Calibration.SHOOTER_SPIN_SPEED_LOW_BAR;
    }
    
    public void setDefaultSpeed() {
        speed = Calibration.SHOOTER_SPIN_SPEED;
    }
    
    private class AutoStop {

        private boolean hasFired = false;
        private long time = -1;

        boolean tick() {
            if (hasBeenSpunUp && Math.abs(pid.getError()) > Calibration.SHOOTER_BALL_ERROR_THRESHOLD && time == -1) {
                hasFired = true;
                time = System.currentTimeMillis();
            }
            
            if (hasFired) { closeGate(); }

            return hasFired && System.currentTimeMillis() > Calibration.SHOOTER_AUTOSTOP_DELAY + time;
        }

        void reset() {
            hasFired = false;
            closeGate();
            time = -1;
        }
    }
}
