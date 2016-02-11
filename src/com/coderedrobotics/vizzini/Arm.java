package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PWMController;
import com.coderedrobotics.vizzini.statics.Calibration;
import com.coderedrobotics.vizzini.statics.Wiring;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 *
 * @author michael
 */
public class Arm {

    // will need to add limit switches and an encoder (maybe)
    private final CANTalon arm;
    private PIDControllerAIAO pidController;
    private final Pickup pickup;
    private final DigitalInput limitSwitch;
    public long lastArmChange = 0;

    private boolean isCalibrating = false;
    private boolean isCalibrated = false;

    public Arm(int armMotorPort, int pickupFrontMotorPort, int pickupRearMotorPort) {
        pickup = new Pickup(pickupFrontMotorPort, pickupRearMotorPort);
        arm = new CANTalon(armMotorPort);
        limitSwitch = new DigitalInput(Wiring.ARM_LIMIT_SWITCH);
        pidController = new PIDControllerAIAO(Calibration.ARM_P, Calibration.ARM_I,
                Calibration.ARM_D, Calibration.ARM_F, arm, (double output) -> {
                    arm.pidWrite(limitSwitch.get() && output > 0 ? 0 : output);
                }, true, "arm");

        arm.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        arm.configPeakOutputVoltage(12, -12);
        arm.setPosition(0);
    }

    public void move(double speed) {
        long now = System.currentTimeMillis();
        long timePassed = now - lastArmChange;
        lastArmChange = now;

        timePassed = Math.min(60, timePassed);

        pidController.setSetpoint(pidController.getSetpoint() + (timePassed * Calibration.ARM_SETPOINT_INCREMENT * speed));
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

        //  SmartDashboard.putNumber("Arm Position Target", targetArmPosition);
        // SmartDashboard.putNumber("Arm Encoder Position", arm.getPosition());
        pickup.tick();

        if (isCalibrating) {
            if (limitSwitch.get()) {
                System.out.println("CALIBRATED!");
                arm.set(0); // stop the arm
                arm.setPosition(0); // set encoders to 0
                pidController.setSetpoint(-1);
                pidController.enable();
//                arm.changeControlMode(TalonControlMode.Position);
                isCalibrating = false;
                isCalibrated = true;
            }
        }
    }

    public void calibrate(boolean recalibrate) {
        if (!isCalibrated || recalibrate) {
            arm.changeControlMode(TalonControlMode.PercentVbus);
            isCalibrating = true;
            arm.set(Calibration.ARM_CALIBRATION_MOTOR_SPEED); // start the arm in downward motion
        }
    }
}
