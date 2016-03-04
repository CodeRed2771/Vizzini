package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PIDSourceFilter;
import com.coderedrobotics.libs.PWMController;
import com.coderedrobotics.libs.PWMSplitter2X;
import com.coderedrobotics.libs.TankDrive;
import com.coderedrobotics.vizzini.statics.Calibration;
import com.coderedrobotics.vizzini.statics.Wiring;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveAuto {

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private PWMSplitter2X leftDrive;
    private PWMSplitter2X rightDrive;

    private PIDControllerAIAO leftDrivePID;
    private PIDControllerAIAO rightDrivePID;
    
    private double targetDistance;
  
    public DriveAuto(Encoder leftEncoder, Encoder rightEncoder, PWMSplitter2X leftDrive, PWMSplitter2X rightDrive) {
     	
    	this.leftEncoder = leftEncoder;
    	this.rightEncoder = rightEncoder;
    	this.rightDrive = rightDrive;
    	this.leftDrive = leftDrive;
    	
     	//this.rightEncoder.setReverseDirection(true);
    	//this.leftEncoder.setReverseDirection(true);
    	
        this.leftEncoder.setDistancePerPulse(Calibration.DRIVE_DISTANCE_PER_PULSE);
    	this.rightEncoder.setDistancePerPulse(Calibration.DRIVE_DISTANCE_PER_PULSE);
    	
        leftDrivePID = new PIDControllerAIAO(.5, Calibration.DRIVE_I, Calibration.DRIVE_D, new PIDSourceFilter((double value) -> -this.leftEncoder.getDistance()), this.leftDrive,  false, "left");
        rightDrivePID = new PIDControllerAIAO(.5, Calibration.DRIVE_I, Calibration.DRIVE_D, new PIDSourceFilter((double value) -> this.rightEncoder.getDistance()), this.rightDrive, false, "right");
        
        leftDrivePID.reset();
        leftDrivePID.setSetpoint(0);
        leftDrivePID.disable();
        rightDrivePID.reset();
        rightDrivePID.setSetpoint(0);
        rightDrivePID.disable();
    }
       
    public void resetEncoders() {
    	rightEncoder.reset();
    	leftEncoder.reset();
    }
    
    public void showEncoderValues() {
    	SmartDashboard.putNumber("Left Drive Encoder Distance: ", leftEncoder.getDistance());
    	SmartDashboard.putNumber("Right Drive Encoder Distance: ", rightEncoder.getDistance());
    	SmartDashboard.putNumber("Right Drive Encoder Raw: ", rightEncoder.getRaw());
      	SmartDashboard.putNumber("Left Drive Encoder Raw: ", leftEncoder.getRaw());
      	   	
      	SmartDashboard.putNumber("Target: ", targetDistance);
    }
    
    public void driveInches(int inches, double maxPower) {
    	//rightDrivePID.setOutputRange(-maxPower, maxPower);
    	//leftDrivePID.setOutputRange(-maxPower, maxPower);
    	
    	leftDrivePID.setSetpoint(inches);
    	rightDrivePID.setSetpoint(inches);
    	
    	leftDrivePID.enable();
    	rightDrivePID.enable();
    	//targetDistance = inches;
    	//leftDrive.set(maxPower);
    	//rightDrive.set(maxPower);
    }
    
    public void updateDriveStatus() {
    	
    }
    
    
    public void stop() {
    	leftDrivePID.disable();
    	rightDrivePID.disable();
    	leftDrive.set(0);
    	rightDrive.set(0);
    }
    
    public boolean hasArrived() {
    	return false;
    	
    	//return ((Math.abs(leftEncoder.getDistance()) > Math.abs(targetDistance)) || (Math.abs(rightEncoder.getDistance()) > Math.abs(targetDistance))); 
    	
    }
    

}
