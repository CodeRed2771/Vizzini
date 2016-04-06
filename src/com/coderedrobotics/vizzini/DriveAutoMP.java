package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.PIDControllerAIAO;
import com.coderedrobotics.libs.PIDSourceFilter;
import com.coderedrobotics.vizzini.statics.Calibration;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveAutoMP {
	   private PIDHolder leftPIDHolder;
	    private PIDHolder rightPIDHolder;

	    private PIDControllerAIAO leftDrivePID;
	    private PIDControllerAIAO rightDrivePID;
	    public PIDControllerAIAO rotDrivePID;
	    private Drive mainDrive;
	    private AnalogGyro gyro;


	    public DriveAutoMP(Drive mainDrive, AnalogGyro gyro) {
	        this.mainDrive = mainDrive;
	        this.gyro = gyro;

	        leftPIDHolder = new PIDHolder();
	        rightPIDHolder = new PIDHolder();
	        

	        leftDrivePID = new PIDControllerAIAO(0, 0, 0, new PIDSourceFilter((double value) -> -mainDrive.getLeftEncoderObject().get()), leftPIDHolder, false, "autoleft");
	        rightDrivePID = new PIDControllerAIAO(0, 0, 0, new PIDSourceFilter((double value) -> -mainDrive.getRightEncoderObject().get()), rightPIDHolder, false, "autoright");
	        rotDrivePID = new PIDControllerAIAO(Calibration.AUTO_GYRO_P, Calibration.AUTO_GYRO_I, Calibration.AUTO_GYRO_D, gyro, (double value) -> mainDrive.set(value, -value), false, "Gyro rot");

	        rightDrivePID.setPID(Calibration.AUTO_DRIVE_P, 0, 0);
	        leftDrivePID.setPID(Calibration.AUTO_DRIVE_P, 0, 0);


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
	        
	        resetEncoders();
	        
	        rightDrivePID.enable();
	        leftDrivePID.enable();
	        
	        
	    }

	
	    public void setPosition(double leftDrive, double rightDrive) {
	        leftDrivePID.setSetpoint(leftDrive);
	        rightDrivePID.setSetpoint(rightDrive);
	        outputToDriveTrain();
	    }

	
	    public double getDistanceTravelled() {
	        return Math.abs(convertTicksToInches(mainDrive.getLeftEncoderObject().get()));
	    }


	    private void outputToDriveTrain() {
	        // this is called from the PIDWrites to send the new output values to the main drive object
	        mainDrive.set(leftPIDHolder.PIDvalue, rightPIDHolder.PIDvalue);
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
	            
	            PIDvalue = output;
	           
	            outputToDriveTrain();
	        }
	    }

	    public void showEncoderValues() {
	        SmartDashboard.putNumber("MP Left Drive PID Avg Error: ", leftDrivePID.getAvgError());
	        SmartDashboard.putNumber("MP Right Drive PID Avg Error: ", rightDrivePID.getAvgError());
	        SmartDashboard.putBoolean("MP Left On Target", leftDrivePID.onTarget());
	        SmartDashboard.putBoolean("MP Right On Target", rightDrivePID.onTarget());
//	        SmartDashboard.putNumber("Gyro", gyro.getAngle());
//	        SmartDashboard.putNumber("Gyro PID error", rotDrivePID.getAvgError());

	        SmartDashboard.putNumber("MP Left Drive Encoder Raw: ", mainDrive.getLeftEncoderObject().get());
	        SmartDashboard.putNumber("MP Right Drive Encoder Raw: ", mainDrive.getRightEncoderObject().get());
	        SmartDashboard.putNumber("MP Left Setpoint: ", leftDrivePID.getSetpoint());
	        SmartDashboard.putNumber("MP Right Setpoint: ", rightDrivePID.getSetpoint());
	       
	        //		SmartDashboard.putNumber("Right PID error", rightDrivePID.getError());
	        //   	SmartDashboard.putNumber("Left Drive Encoder Get: ", mainDrive.getLeftEncoderObject().get());
	        //  	SmartDashboard.putNumber("Right Drive Encoder Get: ", mainDrive.getRightEncoderObject().get());
	        //     	SmartDashboard.putNumber("Left Drive Distance: ", leftEncoder.getDistance());
	        //     	SmartDashboard.putNumber("Right Drive Distance: ", rightEncoder.getDistance());
	         //		SmartDashboard.putNumber("Right Drive Encoder Raw: ", rightEncoder.getRaw());
	        //		SmartDashboard.putNumber("Right Setpoint: ", rightDrivePID.getSetpoint());
	    }

}
