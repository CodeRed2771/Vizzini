package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PIDSourceFilter;
import com.coderedrobotics.libs.RobotLEDs;
import com.coderedrobotics.libs.RobotLEDs.Color;
import com.coderedrobotics.vizzini.statics.Calibration;
import com.coderedrobotics.vizzini.statics.Wiring;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author michael
 */
public class Arm {

    private final CANTalon arm;
    private final PIDControllerAIAO pidController;
    private final Pickup pickup;
    private final DigitalInput limitSwitch;
    public long lastArmChange = 0;
    private final RobotLEDs leds;

    private boolean isCalibrating = false;
    private boolean isCalibrated = false;
    private boolean overrideEnabled = false;

    public Arm(int armMotorPort, int pickupFrontMotorPort, int pickupRearMotorPort, RobotLEDs leds) {
    	this.leds = leds;
        pickup = new Pickup(pickupFrontMotorPort, pickupRearMotorPort, (Object o) -> pickupBallEvent());
        arm = new CANTalon(armMotorPort);

        limitSwitch = new DigitalInput(Wiring.ARM_LIMIT_SWITCH);
        pidController = new PIDControllerAIAO(Calibration.ARM_P, Calibration.ARM_I,
                Calibration.ARM_D, Calibration.ARM_F, new PIDSourceFilter(arm, (double value) -> -arm.pidGet()), (double output) -> {
                    arm.pidWrite(limitSwitch.get() && output > 0 ? 0 : -output);
                }, false, "arm");
        
        pidController.setInputRange(Calibration.ARM_MIN_SETPOINT, Calibration.ARM_MAX_SETPOINT);
        
        pidController.setAbsoluteTolerance(Calibration.ARM_PID_TEST_TOLERANCE);
        arm.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);  // why Relative and not absolute? (Changed to Absolute to fix it - DVV 3/10/16)
        arm.configPeakOutputVoltage(12, -12);  
        arm.enableBrakeMode(true);
        arm.setPosition(0);
    }

    public void move(double speed) {
        if (!overrideEnabled) {
            long now = System.currentTimeMillis();
            long timePassed = now - lastArmChange;
            lastArmChange = now;

            timePassed = Math.min(60, timePassed);
                  
            pidController.setSetpoint(pidController.getSetpoint() + (timePassed * Calibration.ARM_SETPOINT_INCREMENT * speed));
        } else {
            arm.set(-speed);
        }
    }

    public void gotoShootPosition() {
        pidController.setSetpoint(-Calibration.ARM_SHOOT_POSITION);
    }

    public void gotoPickupPosition() {
        pidController.setSetpoint(0);
    }
    
    public Object pickupBallEvent() {
    	gotoRestingPosition();
    	leds.blinkThrice(Color.WHITE, 3);
    	return null;
    }
    
    public void gotoRestingPosition() {
        pidController.setSetpoint(-.4);
    }
    
    public void gotoPortcullisPosition() {
    	pidController.setSetpoint(-.15);
    }
    public void gotoChivalDeFrisePosition(){
    	pidController.setSetpoint(-Calibration.ARM_CHIVAL_POSITION);
    }
    
    public void disablePIDController() {
        overrideEnabled = true;
        pidController.disable();
    }

    public void feedIn() {
        pickup.feedIn();
    }
    
    public void feedInNudge() {
        pickup.feedInNudge();
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
    
    public boolean isCalibrated() {
        return isCalibrated;
    }

    public boolean passedLimitSwitchTest() {
        return isCalibrated && !limitSwitch.get() && Math.abs(pidController.getError()) < 0.2;
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
        
		SmartDashboard.putBoolean("Calibration: ", isCalibrated);

    }

    public void calibrate(boolean recalibrate) {
//        Logger.getInstance().log(Logger.Level.INFO, 13, "CALIBRATE!");
        if (!isCalibrated || recalibrate) {
            arm.changeControlMode(TalonControlMode.PercentVbus);
            isCalibrating = true;
            isCalibrated = false;
            pidController.disable();
            pidController.setSetpoint(0);
            arm.set(-Calibration.ARM_CALIBRATION_MOTOR_SPEED); // start the arm in downward motion
        }
    }
}
