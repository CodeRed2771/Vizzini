package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PWMController;
import com.coderedrobotics.vizzini.statics.Wiring;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

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
    private double targetArmPosition = 0;
    private static final double armIncrement = .1;
    public static final int RED_AND_GREEN_LEDS = 0;
    public static final int BLUE_LEDS = 1;
    
    private boolean isCalibrating = false;
    private boolean isCalibrated = false;
    
    public Arm(int armMotorPort, int pickupFrontMotorPort, int pickupRearMotorPort) {
        pickup = new Pickup(pickupFrontMotorPort, pickupRearMotorPort);
        arm = new CANTalon(armMotorPort);
        limitSwitch = new DigitalInput(Wiring.ARM_LIMIT_SWITCH);
      
        arm.setPID(.10, 0, .002);
        arm.setIZone(0);

        arm.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        arm.configPeakOutputVoltage(12, -12);
        arm.setVoltageRampRate(12);
        arm.setPosition(0);
        
        targetArmPosition = 0;
    }
    
    public void move(double direction) {
        if (direction > 0 && limitSwitch.get()) {
        	direction = 0;
        }
        targetArmPosition += (armIncrement * direction);
        
        arm.set(targetArmPosition); 
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
    	
    	SmartDashboard.putNumber("Arm Position Target", targetArmPosition);
    	SmartDashboard.putNumber("Arm Encoder Position", arm.getPosition());
    	
        pickup.tick();
        
      	if (isCalibrating) {
    		if (limitSwitch.get()) {
    			arm.set(0); // stop the arm
    			arm.setPosition(0); // set encoders to 0
    	        arm.changeControlMode(TalonControlMode.Position);
    			isCalibrating = false;
    			isCalibrated = true;
    			targetArmPosition = 0;
    		}
    	}
    }
    
    public void calibrate(boolean recalibrate) {
    	if (!isCalibrated || recalibrate) {
            arm.changeControlMode(TalonControlMode.PercentVbus);
	    	isCalibrating = true;
	    	arm.set(.3); // start the arm in downward motion
    	}
    }
}
