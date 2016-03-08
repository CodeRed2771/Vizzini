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
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveAuto {

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private PWMSplitter2X leftDrive;
    private PWMSplitter2X rightDrive;

    private PIDControllerAIAO leftDrivePID;
    private PIDControllerAIAO rightDrivePID;
    
    private double myCounter = 0;
    
    public DriveAuto(Encoder leftEncoder, Encoder rightEncoder, PWMSplitter2X leftDrive, PWMSplitter2X rightDrive) {
     	
    	this.leftEncoder = leftEncoder;
    	this.rightEncoder = rightEncoder;
    	this.rightDrive = rightDrive;
    	this.leftDrive = leftDrive;
    	
     	this.rightEncoder.setReverseDirection(true);
    	this.leftEncoder.setReverseDirection(true);
    	
        this.leftEncoder.setDistancePerPulse(Calibration.DRIVE_DISTANCE_PER_PULSE);
    	this.rightEncoder.setDistancePerPulse(Calibration.DRIVE_DISTANCE_PER_PULSE);
    	this.leftEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
    	this.rightEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
    	
        leftDrivePID = new PIDControllerAIAO(.1, 0, 0, new PIDSourceFilter((double value) -> -this.leftEncoder.getDistance()), this.leftDrive,  false, "left");
        rightDrivePID = new PIDControllerAIAO(.1, 0, 0, new PIDSourceFilter((double value) -> -this.rightEncoder.getDistance()), this.rightDrive, false, "right");
 
        
       // leftDrivePID.setPercentTolerance(5); // this value hasn't really been validated yet
       // rightDrivePID.setPercentTolerance(5);
        leftDrivePID.setAbsoluteTolerance(2);
        rightDrivePID.setAbsoluteTolerance(2);
        leftDrivePID.setToleranceBuffer(10);
        rightDrivePID.setToleranceBuffer(10);
        leftDrivePID.setSetpoint(0);
        leftDrivePID.reset();
        rightDrivePID.setSetpoint(0);
        rightDrivePID.reset();
        
    }
       
    public void resetEncoders() {
    	rightEncoder.reset();
    	leftEncoder.reset();
    }
   
    public void driveInches(int inches, double maxPower) {
    	rightDrivePID.setOutputRange(-maxPower, maxPower);
    	leftDrivePID.setOutputRange(-maxPower, maxPower);
    	
    	leftDrivePID.setSetpoint(inches);
    	rightDrivePID.setSetpoint(inches);
    	
    	leftDrivePID.enable();
    	rightDrivePID.enable();
    	//myCounter = 0;
    }
   
    public void turnDegrees(int degrees, double maxPower) {
    	
    	double inchesToTravel = degrees/6;

    	rightDrivePID.setOutputRange(-maxPower, maxPower);
    	leftDrivePID.setOutputRange(-maxPower, maxPower);
    	
		leftDrivePID.setSetpoint(inchesToTravel);
		rightDrivePID.setSetpoint(-inchesToTravel);
      	
    	leftDrivePID.enable();
    	rightDrivePID.enable();

    }
    
    public void updateDriveStatus() {
    	
    }
    
    public void stop() {
    	leftDrivePID.disable();
    	rightDrivePID.disable();
    }
    
    public boolean hasArrived() {
    	return leftDrivePID.onTarget() && rightDrivePID.onTarget();
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
      
    public void showEncoderValues() {
    	//SmartDashboard.putNumber("Left Drive Encoder Distance: ", leftEncoder.getDistance());
    	//SmartDashboard.putNumber("Right Drive Encoder Distance: ", rightEncoder.getDistance());
    	//SmartDashboard.putNumber("Right PID error", rightDrivePID.getError());
     	SmartDashboard.putNumber("Left Drive PID Avg Error: ", leftDrivePID.getAvgError());
     	
     	SmartDashboard.putNumber("Left PID error", leftDrivePID.getError());
     	SmartDashboard.putNumber("Left Drive Encoder Raw: ", leftEncoder.getRaw());
     	SmartDashboard.putNumber("Left Drive Distance: ", leftEncoder.getDistance());
      	SmartDashboard.putNumber("Left Setpoint: ", leftDrivePID.getSetpoint());

      	SmartDashboard.putBoolean("On Target", leftDrivePID.onTarget());
      	
     	//SmartDashboard.putNumber("Right Drive Encoder Raw: ", rightEncoder.getRaw());
       	   	
      	//SmartDashboard.putNumber("Right Setpoint: ", rightDrivePID.getSetpoint());
    }
    
}
