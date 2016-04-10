package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PIDSourceFilter;
import com.coderedrobotics.vizzini.statics.Calibration;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveAuto {

    private PIDHolder leftPIDHolder;
    private PIDHolder rightPIDHolder;

    private PIDControllerAIAO leftDrivePID;
    private PIDControllerAIAO rightDrivePID;
    public PIDControllerAIAO rotDrivePID;
    private Drive mainDrive;
    private AnalogGyro gyro;

    private boolean drivingStraight = true;

    private double maxPowerAllowed = 1;
    private double curPowerSetting = 1;
    private long timeDriveStarted = 0;

    public DriveAuto(Drive mainDrive, AnalogGyro gyro) {
        this.mainDrive = mainDrive;
        this.gyro = gyro;

        leftPIDHolder = new PIDHolder();
        rightPIDHolder = new PIDHolder();

        leftDrivePID = new PIDControllerAIAO(0, 0, 0, new PIDSourceFilter((double value) -> -mainDrive.getLeftEncoderObject().get()), leftPIDHolder, false, "autoleft");
        rightDrivePID = new PIDControllerAIAO(0, 0, 0, new PIDSourceFilter((double value) -> -mainDrive.getRightEncoderObject().get()), rightPIDHolder, false, "autoright");
        rotDrivePID = new PIDControllerAIAO(Calibration.AUTO_GYRO_P, Calibration.AUTO_GYRO_I, Calibration.AUTO_GYRO_D, gyro, (double value) -> mainDrive.set(value, -value), false, "Gyro rot");

        leftDrivePID.setAbsoluteTolerance(Calibration.DRIVE_DISTANCE_TICKS_PER_INCH / 2); // half inch
        rightDrivePID.setAbsoluteTolerance(Calibration.DRIVE_DISTANCE_TICKS_PER_INCH / 2);
        rotDrivePID.setAbsoluteTolerance(1.2);
        rotDrivePID.setToleranceBuffer(10);
        
        leftDrivePID.setToleranceBuffer(20); // ten readings
        rightDrivePID.setToleranceBuffer(20);
        leftDrivePID.setSetpoint(0);
        leftDrivePID.reset();
        rightDrivePID.setSetpoint(0);
        rightDrivePID.reset();
        
    }

    public void driveInches(int inches, double maxPower) {

        stop();

        gyro.reset();
        maxPowerAllowed = maxPower;
        curPowerSetting = .1;  // the minimum power required to start moving.  (Untested)

        rightDrivePID.disable();
        leftDrivePID.disable();
        rotDrivePID.disable();

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

    public void test() {
        rightDrivePID.disable();
        leftDrivePID.disable();
        mainDrive.set(0.8, -.8);
    }

   
    public void turnDegreesFromZero(double degrees, double maxPower) {
       	// Turns using the Gyro, relative to the ZERO position
    	// Use "turnCompleted" method to determine when the turn is done
    	
    	// NOTE - THIS WAS NOT WORKING AS EXPECTED WHEN TURNING MORE THAN ONCE
    	
    	SmartDashboard.putNumber("TURN FROM ZERO CALL", degrees);
    	     	
    	maxPowerAllowed = maxPower;
    	curPowerSetting = .18;
    	
        leftDrivePID.disable();
        rightDrivePID.disable();
        drivingStraight = false;
        
        rotDrivePID.setSetpoint(degrees);
        rotDrivePID.enable();
        
        setPowerOutput(curPowerSetting);
        												
    }
    
    public void turnDegrees(int degrees, double maxPower) {
    	// Turns using the Gyro, relative to the current position
    	// Use "turnCompleted" method to determine when the turn is done

    	SmartDashboard.putNumber("TURN DEGREES CALL", degrees);
    	
    	maxPowerAllowed = maxPower;
       	curPowerSetting = .18;
            	
        leftDrivePID.disable();
        rightDrivePID.disable();
        drivingStraight = false;
        rotDrivePID.setSetpoint(gyro.getAngle() + degrees);
        rotDrivePID.enable();
        
        setPowerOutput(curPowerSetting);

    }

    public void tick() {
    	// this is called roughly 50 times per second
    	
    	// check for ramping up
        if (curPowerSetting < maxPowerAllowed) {  // then increase power a notch 
            curPowerSetting += .015; // was .007 evening of 4/5 // to figure out how fast this would be, multiply by 50 to see how much it would increase in 1 second.
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
        rightDrivePID.setOutputRange(-powerLevel, powerLevel);
        leftDrivePID.setOutputRange(-powerLevel, powerLevel);
        rotDrivePID.setOutputRange(-powerLevel, powerLevel);
    }

    public void setMaxPowerOutput(double maxPower) {
        maxPowerAllowed = maxPower;
        // "tick" will take care of implementing this power level
    }

    public void addInches(int inches) {
        leftDrivePID.setSetpoint(leftDrivePID.getSetpoint() + convertToTicks(inches));
        rightDrivePID.setSetpoint(rightDrivePID.getSetpoint() + convertToTicks(inches));
    }

    public double getDistanceTravelled() {
        return Math.abs(convertTicksToInches(mainDrive.getLeftEncoderObject().get()));
    }

    public void stop() {
        leftDrivePID.disable();
        rightDrivePID.disable();
        rotDrivePID.disable();
        mainDrive.set(0, 0);
    }

    public boolean hasArrived() {
        return leftDrivePID.onTarget() && rightDrivePID.onTarget();
    }

    public boolean turnCompleted() {
        return rotDrivePID.onTarget();
    }
  
    private void outputToDriveTrain() {
        // this is called from the PIDWrites to send the new output values to the main drive object
        mainDrive.set(leftPIDHolder.PIDvalue - alignmentAdjust() , rightPIDHolder.PIDvalue  );
    }

    private double alignmentAdjust() {
        if (drivingStraight)   	{
        	double adjustAmt = (mainDrive.getRightEncoderObject().get() - mainDrive.getLeftEncoderObject().get()) * .03;  // was .02 4/5/16    .04 is too high   
        	SmartDashboard.putNumber("DriveStraight Adjustment", adjustAmt);
        	return adjustAmt;
        }
        else {
            return 0;
        }
        
        //        if (drivingStraight) {
//            if (gyro.getAngle() < 180) {
//                return (gyro.getAngle() * .08);
//            } else {
//                return (gyro.getAngle() - 360 * .08); // was .1, .2 (drove jumpy), .15
//            }
//        } 


    }

   private void resetEncoders() {
        mainDrive.getLeftEncoderObject().reset();
        mainDrive.getRightEncoderObject().reset();
    }

    public void setPIDstate(boolean isEnabled) {
        if (isEnabled) {
            leftDrivePID.enable();
            rightDrivePID.enable();
            rotDrivePID.enable();
        } else {
            leftDrivePID.disable();
            rightDrivePID.disable();
            rotDrivePID.disable();
        }
    }

    private int convertToTicks(int inches) {
        return (int) (inches * Calibration.DRIVE_DISTANCE_TICKS_PER_INCH);
    }

    private int convertToTicks(double inches) {
        return (int) (inches * Calibration.DRIVE_DISTANCE_TICKS_PER_INCH);
    }

    private double convertTicksToInches(int ticks) {
        return ticks / Calibration.DRIVE_DISTANCE_TICKS_PER_INCH;
    }

    private class PIDHolder implements PIDOutput {

        public double PIDvalue = 0;

        @Override
        public void pidWrite(double output) {
            if (drivingStraight) {
                PIDvalue = output;
            } else {
                PIDvalue = output;
            }

            outputToDriveTrain();
        }
    }

    public void showEncoderValues() {
        SmartDashboard.putNumber("Left Drive PID Avg Error: ", leftDrivePID.getAvgError());
        SmartDashboard.putNumber("Right Drive PID Avg Error: ", rightDrivePID.getAvgError());
        SmartDashboard.putBoolean("Left On Target", leftDrivePID.onTarget());
        SmartDashboard.putBoolean("Right On Target", rightDrivePID.onTarget());
        SmartDashboard.putNumber("Gyro", gyro.getAngle());
        SmartDashboard.putNumber("Gyro PID error", rotDrivePID.getAvgError());

        SmartDashboard.putNumber("Left Drive Encoder Raw: ", mainDrive.getLeftEncoderObject().get());
        SmartDashboard.putNumber("Right Drive Encoder Raw: ", mainDrive.getRightEncoderObject().get());
        SmartDashboard.putNumber("Left Setpoint: ", leftDrivePID.getSetpoint());
        SmartDashboard.putNumber("Right Setpoint: ", rightDrivePID.getSetpoint());
       
        //		SmartDashboard.putNumber("Right PID error", rightDrivePID.getError());
        //   	SmartDashboard.putNumber("Left Drive Encoder Get: ", mainDrive.getLeftEncoderObject().get());
        //  	SmartDashboard.putNumber("Right Drive Encoder Get: ", mainDrive.getRightEncoderObject().get());
        //     	SmartDashboard.putNumber("Left Drive Distance: ", leftEncoder.getDistance());
        //     	SmartDashboard.putNumber("Right Drive Distance: ", rightEncoder.getDistance());
         //		SmartDashboard.putNumber("Right Drive Encoder Raw: ", rightEncoder.getRaw());
        //		SmartDashboard.putNumber("Right Setpoint: ", rightDrivePID.getSetpoint());
    }

}
