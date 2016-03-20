package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.Logger;
import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PIDSourceFilter;
import com.coderedrobotics.libs.PWMController;
import com.coderedrobotics.libs.PWMSplitter2X;
import com.coderedrobotics.libs.TankDrive;
import com.coderedrobotics.vizzini.statics.Calibration;
import com.coderedrobotics.vizzini.statics.Wiring;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveAuto {

	private PIDHolder leftPIDHolder;
	private PIDHolder rightPIDHolder;

    private PIDControllerAIAO leftDrivePID;
    private PIDControllerAIAO rightDrivePID;
    private Drive mainDrive;
    
    private boolean drivingStraight = true;
    
    private double maxPowerAllowed = 1;
    private double curPowerSetting = 1;
    private long timeDriveStarted = 0;
        
    public DriveAuto(Drive mainDrive) {
     	this.mainDrive = mainDrive;
     	leftPIDHolder = new PIDHolder();
     	rightPIDHolder = new PIDHolder();
     	
        leftDrivePID = new PIDControllerAIAO(0, 0, 0, new PIDSourceFilter((double value) -> -mainDrive.getLeftEncoderObject().get()), leftPIDHolder,  false, "autoleft");
        rightDrivePID = new PIDControllerAIAO(0, 0, 0, new PIDSourceFilter((double value) -> -mainDrive.getRightEncoderObject().get()), rightPIDHolder, false, "autoright");
 
        leftDrivePID.setAbsoluteTolerance(Calibration.DRIVE_DISTANCE_TICKS_PER_INCH / 2); // half inch
        rightDrivePID.setAbsoluteTolerance(Calibration.DRIVE_DISTANCE_TICKS_PER_INCH/ 2);
        leftDrivePID.setToleranceBuffer(20); // ten readings
        rightDrivePID.setToleranceBuffer(20);
        leftDrivePID.setSetpoint(0);
        leftDrivePID.reset();
        rightDrivePID.setSetpoint(0);
        rightDrivePID.reset();
        
    }
   
    public void driveInches(int inches, double maxPower) {
    	
    	stop();
    	
    	maxPowerAllowed = maxPower;
    	curPowerSetting = .1;
       	rightDrivePID.disable();
    	leftDrivePID.disable();
   	
    	setPowerOutput(curPowerSetting);
    	
    	resetEncoders();
    	drivingStraight = true;   
    	
    	rightDrivePID.setPID(Calibration.AUTO_DRIVE_P, 0, 0);
    	leftDrivePID.setPID(Calibration.AUTO_DRIVE_P, 0, 0);
    	
    	rightDrivePID.setSetpoint(-mainDrive.getRightEncoderObject().get() + convertToTicks(inches));
    	leftDrivePID.setSetpoint(-mainDrive.getLeftEncoderObject().get() + convertToTicks(inches));
    	
    	rightDrivePID.enable();
    	leftDrivePID.enable();
    	
    	timeDriveStarted = System.currentTimeMillis();
    	
    }
    
    public void curveDrive(int inches, double ratio) {
    	
    }
    
    public void turnDegrees(int degrees, double maxPower) {
    	
    	double inchesToTravel = degrees/6.6;
       	
    	stop();
    	
    	maxPowerAllowed = maxPower;
    	curPowerSetting = .18;
         	
      	rightDrivePID.disable();
    	leftDrivePID.disable();
 
    	setPowerOutput(curPowerSetting);
    	
    	resetEncoders();
       	drivingStraight = false;
            	
      	rightDrivePID.setPID(Calibration.AUTO_TURN_P, 0, .01);
    	leftDrivePID.setPID(Calibration.AUTO_TURN_P, 0, .01);

    	leftDrivePID.setSetpoint(-mainDrive.getLeftEncoderObject().get() + convertToTicks(inchesToTravel));	
		rightDrivePID.setSetpoint(-mainDrive.getRightEncoderObject().get() + convertToTicks(-inchesToTravel));
      	
    	leftDrivePID.enable();
    	rightDrivePID.enable();

    	timeDriveStarted = System.currentTimeMillis();
   }
    
    public void tick() {
    	if (curPowerSetting < maxPowerAllowed) {  // then increase power a notch 
			curPowerSetting += .005;
			if (curPowerSetting > maxPowerAllowed) curPowerSetting = maxPowerAllowed;
			setPowerOutput(curPowerSetting);
    	}
    }
    
    private void setPowerOutput(double maxPower) {
       	rightDrivePID.setOutputRange(-maxPower, maxPower);
    	leftDrivePID.setOutputRange(-maxPower, maxPower);
    }
    
    public void setMaxPowerOutput(double maxPower) {
    	maxPowerAllowed = maxPower;
    }
    
    public void addInches(int inches) {
    	leftDrivePID.setSetpoint(leftDrivePID.getSetpoint() + convertToTicks(inches));
    	rightDrivePID.setSetpoint(rightDrivePID.getSetpoint() + convertToTicks(inches));
    }
    
    public double getDistanceTravelled() {
    	return Math.abs(convertTicksToInches(mainDrive.getLeftEncoderObject().get())); 
    }
    
    private double encoderAdjust() {
    	if (drivingStraight) 
    		return (mainDrive.getRightEncoderObject().get() - mainDrive.getLeftEncoderObject().get()) * .02;
    	else
    		return 0;
    }
    
    private void outputToDriveTrain() {
    	// this is called from the PIDWrites to send the new output values to the main drive object
    	mainDrive.set(leftPIDHolder.PIDvalue, rightPIDHolder.PIDvalue + encoderAdjust());
    }
    
   public void stop() {
    	leftDrivePID.disable();
    	rightDrivePID.disable();
    	mainDrive.set(0, 0);
    	
    }
    
    public boolean hasArrived() {
    	return leftDrivePID.onTarget() && rightDrivePID.onTarget();
    }
    
    private void resetEncoders() {
    	mainDrive.getLeftEncoderObject().reset();
    	mainDrive.getRightEncoderObject().reset();
    }
    
    public void setPIDstate(boolean isEnabled) {
    	if (isEnabled) {
    		leftDrivePID.enable();
    		rightDrivePID.enable();
    	} else
    	{
    		leftDrivePID.disable();
    		rightDrivePID.disable();
    	}
    }
      
    private int convertToTicks(int inches) {
    	return (int)(inches * Calibration.DRIVE_DISTANCE_TICKS_PER_INCH);
    }
    private int convertToTicks(double inches) {
    	return (int)(inches * Calibration.DRIVE_DISTANCE_TICKS_PER_INCH);
    }
    
    private double convertTicksToInches(int ticks) {
    	return ticks / Calibration.DRIVE_DISTANCE_TICKS_PER_INCH;
    }
   
	private class PIDHolder implements PIDOutput {
		public double PIDvalue = 0;
		
		@Override
		public void pidWrite(double output) {
			PIDvalue = output;
			outputToDriveTrain();
		}
	}
	   
    public void showEncoderValues() {
    	//SmartDashboard.putNumber("Left Drive Encoder Distance: ", leftEncoder.getDistance());
    	//SmartDashboard.putNumber("Right Drive Encoder Distance: ", rightEncoder.getDistance());
    	//SmartDashboard.putNumber("Right PID error", rightDrivePID.getError());
     	SmartDashboard.putNumber("Left Drive PID Avg Error: ", leftDrivePID.getAvgError());
      	SmartDashboard.putNumber("Right Drive PID Avg Error: ", rightDrivePID.getAvgError());

      	SmartDashboard.putNumber("Left Drive Encoder Get: ", mainDrive.getLeftEncoderObject().get());
     	SmartDashboard.putNumber("Right Drive Encoder Get: ", mainDrive.getRightEncoderObject().get());
     	
//     	SmartDashboard.putNumber("Left Drive Distance: ", leftEncoder.getDistance());
//     	SmartDashboard.putNumber("Right Drive Distance: ", rightEncoder.getDistance());
      	
     	SmartDashboard.putNumber("Left Setpoint: ", leftDrivePID.getSetpoint());
      	SmartDashboard.putNumber("Right Setpoint: ", rightDrivePID.getSetpoint());
      	
     	SmartDashboard.putBoolean("Left On Target", leftDrivePID.onTarget());
        SmartDashboard.putBoolean("Right On Target", rightDrivePID.onTarget());
      	
     	//SmartDashboard.putNumber("Right Drive Encoder Raw: ", rightEncoder.getRaw());
       	   	
      	//SmartDashboard.putNumber("Right Setpoint: ", rightDrivePID.getSetpoint());
    }
   
}