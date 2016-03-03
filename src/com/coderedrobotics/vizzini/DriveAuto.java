package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
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

    private PIDControllerAIAO leftDrivePID;
    private PIDControllerAIAO rightDrivePID;
  
    public DriveAuto(Encoder leftEncoder, Encoder rightEncoder, PWMController leftDrive, PWMController rightDrive) {
     	
    	this.leftEncoder = leftEncoder;
    	this.rightEncoder = rightEncoder;
    	
        this.leftEncoder.setDistancePerPulse(Calibration.DRIVE_DISTANCE_PER_PULSE);
    	this.rightEncoder.setDistancePerPulse(Calibration.DRIVE_DISTANCE_PER_PULSE);
    	
        leftDrivePID = new PIDControllerAIAO(Calibration.DRIVE_P, Calibration.DRIVE_I, Calibration.DRIVE_D, this.leftEncoder, leftDrive, false, "left");
        rightDrivePID = new PIDControllerAIAO(Calibration.DRIVE_P, Calibration.DRIVE_I, Calibration.DRIVE_D, this.rightEncoder, rightDrive, false, "right");
        
        leftDrivePID.reset();
        leftDrivePID.setSetpoint(0);
        leftDrivePID.enable();
        rightDrivePID.reset();
        rightDrivePID.setSetpoint(0);
        rightDrivePID.enable();
    }
       
    public void resetEncoders() {
    	rightEncoder.reset();
    	leftEncoder.reset();
    }
    
    public void showEncoderValues() {
    	SmartDashboard.putNumber("Left Drive Encoder Distance: ", leftEncoder.getDistance());
    	SmartDashboard.putNumber("Right Drive Encoder Distance: ", rightEncoder.getDistance());
    	SmartDashboard.putNumber("Right Drive Encoder Raw: ", leftEncoder.getRaw());
    }
    
    public void driveInches(int inches, double maxOutputPercent) {
    	rightDrivePID.setOutputRange(0, maxOutputPercent);
    	leftDrivePID.setOutputRange(0, maxOutputPercent);
    	
    	leftDrivePID.setSetpoint(inches);
    	rightDrivePID.setSetpoint(inches);

    }

}
