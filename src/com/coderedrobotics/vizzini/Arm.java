package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PIDSourceFilter;
import com.coderedrobotics.vizzini.statics.Calibration;
import com.coderedrobotics.vizzini.statics.Wiring;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

/**
 *
 * @author michael
 */
public class Arm {

    private final CANTalon arm;
    private PIDControllerAIAO pidController;
    private final Pickup pickup;
    private final DigitalInput limitSwitch;
    public long lastArmChange = 0;

    private boolean isCalibrating = false;
    private boolean isCalibrated = false;
    private boolean overrideEnabled = false;

    public Arm(int armMotorPort, int pickupFrontMotorPort, int pickupRearMotorPort) {
        pickup = new Pickup(pickupFrontMotorPort, pickupRearMotorPort);
        arm = new CANTalon(armMotorPort);

        limitSwitch = new DigitalInput(Wiring.ARM_LIMIT_SWITCH);
        pidController = new PIDControllerAIAO(Calibration.ARM_P, Calibration.ARM_I,
                Calibration.ARM_D, Calibration.ARM_F, new PIDSourceFilter(arm, (double value) -> -arm.pidGet()), (double output) -> {
                    arm.pidWrite(limitSwitch.get() && output > 0 ? 0 : -output);
                }, true, "arm");

        arm.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);  // why Relative and not absolute?
        arm.configPeakOutputVoltage(12, -12);  // I reduced these from 12 until we resolve the weird problems 2/17/16 DVV
        arm.setPosition(0);
    }

    public void move(double speed) {
        if (!overrideEnabled) {
            long now = System.currentTimeMillis();
            long timePassed = now - lastArmChange;
            lastArmChange = now;

            timePassed = Math.min(60, timePassed);

<<<<<<< HEAD
            pidController.setSetpoint(pidController.getSetpoint() + (timePassed * Calibration.ARM_SETPOINT_INCREMENT * speed));
        } else {
            arm.set(speed);
        }
    }

    public void gotoShootPosition() {
        pidController.setSetpoint(-Calibration.ARM_SHOOT_POSITION);
    }

    public void gotoPickupPosition() {
        pidController.setSetpoint(0);
=======
        pidController.setSetpoint(Math.max(Calibration.ARM_MIN_SETPOINT, Math.min(Calibration.ARM_MAX_SETPOINT, pidController.getSetpoint() + (timePassed * Calibration.ARM_SETPOINT_INCREMENT * speed))));
    }

    public void gotoShootPosition() {
        // Uses PID Controller to move arm to position set in Calibration
    	arm.setPosition(Calibration.ARM_SHOOT_POSITION);
>>>>>>> ded95a3e182c8685e84aa86dd65c6a80f5d8c58a
    }

    public void gotoPickupPosition() {
    	arm.setPosition(0);
    }
    
    public void disablePIDController() {
        overrideEnabled = true;
    }

    public void feedIn() {
        pickup.feedIn();
    }

    public void feedOut() {
        pickup.feedOut();
    }

    public void pickupAllStop() {
        pickup.allStop();
    }

    public void dropBallInShooter() {
        pickup.dropBallInShooter();
    }

    public void stopRearPickupWheels() {
        pickup.stopShooterTriggerWheels();
    }

    public void tick() {

        //  SmartDashboard.putNumber("Arm Position Target", targetArmPosition);
        // SmartDashboard.putNumber("Arm Encoder Position", arm.getPosition());
        pickup.tick();

//        Logger.getInstance().log(Logger.Level.INFO, 222, String.valueOf(arm.pidGet()));
        if (isCalibrating) {
            if (limitSwitch.get()) {
//                Logger.getInstance().log("CALIBRATED!");
                arm.set(0); // stop the arm
                arm.setPosition(0); // set encoders to 0
//                pidController.setSetpoint(-1);
                pidController.enable();
//                arm.changeControlMode(TalonControlMode.Position);
                isCalibrating = false;
                isCalibrated = true;
            }
        }
    }

    public void calibrate(boolean recalibrate) {
//        Logger.getInstance().log(Logger.Level.INFO, 13, "CALIBRATE!");
        if (!isCalibrated || recalibrate) {
            arm.changeControlMode(TalonControlMode.PercentVbus);
            isCalibrating = true;
            arm.set(-Calibration.ARM_CALIBRATION_MOTOR_SPEED); // start the arm in downward motion
        }
    }
}
