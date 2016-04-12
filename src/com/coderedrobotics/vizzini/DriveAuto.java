package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PIDSourceFilter;
import com.coderedrobotics.vizzini.statics.Calibration;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveAuto {

	public PIDControllerAIAO drivePID;
    public PIDControllerAIAO rotDrivePID;
    private Drive mainDrive;
    private AnalogGyro gyro;

    private double maxPowerAllowed = 1;
    private double curPowerSetting = 1;

    public DriveAuto(Drive mainDrive, AnalogGyro gyro) {
        this.mainDrive = mainDrive;
        this.gyro = gyro;

        drivePID = new PIDControllerAIAO(
        		0, 0, 0, new PIDSourceFilter((double value) -> -(mainDrive.getLeftEncoderObject().get() + mainDrive.getRightEncoderObject().get())/2), speed -> mainDrive.autoSetDrive(speed), false, "autodrive");
        rotDrivePID = new PIDControllerAIAO(Calibration.AUTO_GYRO_P, Calibration.AUTO_GYRO_I, Calibration.AUTO_GYRO_D, gyro, rot -> mainDrive.autoSetRot(rot), false, "autorot (gyro)");

        drivePID.setAbsoluteTolerance(Calibration.DRIVE_DISTANCE_TICKS_PER_INCH / 2); // half inch
        rotDrivePID.setAbsoluteTolerance(1.2);
        
        rotDrivePID.setToleranceBuffer(1);        
        drivePID.setToleranceBuffer(1); // ten readings
        
        drivePID.setSetpoint(0);
        drivePID.reset();        
    }

    public void driveInches(int inches, double maxPower) {
        maxPowerAllowed = maxPower;
        curPowerSetting = .1;  // the minimum power required to start moving.  (Untested)

        setPowerOutput(curPowerSetting);

        //drivePID.setPID(Calibration.AUTO_DRIVE_P, 0, 0);
        drivePID.setSetpoint(drivePID.getSetpoint() + convertToTicks(inches));
    }

    public void reset() {
    	drivePID.reset();
    	drivePID.setSetpoint(0);
    	rotDrivePID.reset();
    	rotDrivePID.setSetpoint(0);
    	gyro.reset();
    	mainDrive.getLeftEncoderObject().reset();
    	mainDrive.getRightEncoderObject().reset();
    }
    
    public void turnDegrees(double degrees, double maxPower) {
    	// Turns using the Gyro, relative to the current position
    	// Use "turnCompleted" method to determine when the turn is done

    	SmartDashboard.putNumber("TURN DEGREES CALL", degrees);
    	
    	maxPowerAllowed = maxPower;
       	curPowerSetting = .18;
            	
        rotDrivePID.setSetpoint(rotDrivePID.getSetpoint() + degrees);
        
        setPowerOutput(curPowerSetting);
    }

    public void tick() {
    	// this is called roughly 50 times per second
    	
    	// check for ramping up
        if (curPowerSetting < maxPowerAllowed) {  // then increase power a notch 
            curPowerSetting += .02; // was .007 evening of 4/5 // to figure out how fast this would be, multiply by 50 to see how much it would increase in 1 second.
            if (curPowerSetting > maxPowerAllowed) {
            	curPowerSetting = maxPowerAllowed;
            }
        }
        // now check if we're ramping down
        if (curPowerSetting > maxPowerAllowed) {
        	curPowerSetting -= .03; 
        	if (curPowerSetting < 0) {
        		curPowerSetting = 0;
        	}
        }
        setPowerOutput(curPowerSetting);
        
        SmartDashboard.putNumber("CurPower", curPowerSetting);
        
        // Code from Wednesday night - 4/6/16
        
//        if (curPowerSetting < maxPowerAllowed) {  // then increase power a notch 
//            curPowerSetting += .015; // was .007 evening of 4/5 // to figure out how fast this would be, multiply by 50 to see how much it would increase in 1 second.
//            SmartDashboard.putNumber("CurPower", curPowerSetting);
//            if (curPowerSetting > maxPowerAllowed) {
//                curPowerSetting = maxPowerAllowed;
//            }
//            setPowerOutput(curPowerSetting);
//        }
        
    }

    private void setPowerOutput(double powerLevel) {
        drivePID.setOutputRange(-powerLevel, powerLevel);
        rotDrivePID.setOutputRange(-powerLevel, powerLevel);
    }

    public void setMaxPowerOutput(double maxPower) {
        maxPowerAllowed = maxPower;
        // "tick" will take care of implementing this power level
    }

    public double getDistanceTravelled() {
        return Math.abs(convertTicksToInches(mainDrive.getLeftEncoderObject().get()));
    }

    public boolean hasArrived() {
        return drivePID.onTarget() && rotDrivePID.onTarget();
    }

    public boolean turnCompleted() {
        return hasArrived();
    }

    public void setPIDstate(boolean isEnabled) {
        if (isEnabled) {
            drivePID.enable();
            rotDrivePID.enable();
        } else {
            drivePID.disable();
            rotDrivePID.disable();
        }
    }

    private int convertToTicks(double inches) {
        return (int) (inches * Calibration.DRIVE_DISTANCE_TICKS_PER_INCH);
    }

    private double convertTicksToInches(int ticks) {
        return ticks / Calibration.DRIVE_DISTANCE_TICKS_PER_INCH;
    }

    public void showEncoderValues() {
        SmartDashboard.putNumber("Left Drive PID Avg Error: ", drivePID.getAvgError());
        SmartDashboard.putBoolean("Left On Target", drivePID.onTarget());
        SmartDashboard.putNumber("Gyro", gyro.getAngle());
        SmartDashboard.putNumber("Gyro PID error", rotDrivePID.getAvgError());

        SmartDashboard.putNumber("Left Drive Encoder Raw: ", mainDrive.getLeftEncoderObject().get());
        SmartDashboard.putNumber("Right Drive Encoder Raw: ", mainDrive.getRightEncoderObject().get());
        SmartDashboard.putNumber("Left Setpoint: ", drivePID.getSetpoint());
       
        //		SmartDashboard.putNumber("Right PID error", rightDrivePID.getError());
        //   	SmartDashboard.putNumber("Left Drive Encoder Get: ", mainDrive.getLeftEncoderObject().get());
        //  	SmartDashboard.putNumber("Right Drive Encoder Get: ", mainDrive.getRightEncoderObject().get());
        //     	SmartDashboard.putNumber("Left Drive Distance: ", leftEncoder.getDistance());
        //     	SmartDashboard.putNumber("Right Drive Distance: ", rightEncoder.getDistance());
         //		SmartDashboard.putNumber("Right Drive Encoder Raw: ", rightEncoder.getRaw());
        //		SmartDashboard.putNumber("Right Setpoint: ", rightDrivePID.getSetpoint());
    }

}
