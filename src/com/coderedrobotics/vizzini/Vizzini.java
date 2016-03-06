package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.Logger;
import com.coderedrobotics.libs.RobotLEDs;
import edu.wpi.first.wpilibj.IterativeRobot;
import com.coderedrobotics.vizzini.statics.KeyMap;
import com.coderedrobotics.vizzini.statics.Wiring;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Vizzini extends IterativeRobot {

    TestManager testManager;
    KeyMap keyMap;
    Arm arm;
    Shooter shooter;
    Drive drive;
    DriveAuto driveAuto;
    RobotLEDs leds;
    PowerDistributionPanel pdp;
    SendableChooser chooser;
    final String lowbarAuto = "Low Bar";
    final String touchAuto = "Touch Defense Auto";
    String autoSelected;
    
    int autoStep = 0;

    boolean firing = false;
    private int testStage = 0;
    private long testTimer = 0;
    private boolean hasError = false;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
    	testManager = new TestManager();
    	testManager.addStage(() -> {
    		drive.set(0.2, 0.2);
    		return drive.encoderHasError() ? TestResult.FAILURE : TestResult.INCONCLUSIVE;
    	}, () -> { drive.set(0, 0); }, 1000, TestResult.SUCESS, "drive");
    	testManager.addStage(() -> {
    		arm.calibrate(true);
    		return arm.isCalibrated() ? TestResult.SUCESS : TestResult.INCONCLUSIVE;
    	}, () -> { }, 10000, TestResult.FAILURE, "arm calibration");
    	testManager.addStage(() -> {
    		arm.gotoRestingPosition();
    		return arm.passedLimitSwitchTest() ? TestResult.SUCESS : TestResult.INCONCLUSIVE;
    	}, () -> { }, 3000, TestResult.FAILURE, "arm");
    	testManager.addStage(() -> {
    		shooter.spinUp();
    		return shooter.isSpunUp() ? TestResult.SUCESS : TestResult.INCONCLUSIVE;
    	}, () -> { shooter.stop(); }, 5000, TestResult.FAILURE, "shooter");
    	testManager.addStage(() -> {
    		arm.feedIn();
    		return pdp.getCurrent(Wiring.PICKUP_FRONT_PDP) > 2 ? TestResult.SUCESS : TestResult.INCONCLUSIVE;
    	}, () -> { arm.pickupAllStop(); }, 1000, TestResult.FAILURE, "pickup front");
    	testManager.addStage(() -> {
    		arm.dropBallInShooter();
    		return pdp.getCurrent(Wiring.PICKUP_REAR_PDP) > 2 ? TestResult.SUCESS : TestResult.INCONCLUSIVE;
    	}, () -> { arm.pickupAllStop(); }, 1000, TestResult.FAILURE, "pickup rear");
    	
        keyMap = new KeyMap();
        arm = new Arm(Wiring.ARM_MOTOR, Wiring.PICKUP_FRONT_MOTOR, Wiring.PICKUP_REAR_MOTOR);
        drive = new Drive();
       	driveAuto = new DriveAuto(drive.getLeftEncoderObject(), drive.getRightEncoderObject(), drive.getLeftPWM(), drive.getRightPWM());
        leds = new RobotLEDs(Wiring.RED_AND_GREEN_LEDS, Wiring.BLUE_LEDS);
        shooter = new Shooter(Wiring.SHOOTER_MOTOR_1, Wiring.SHOOTER_MOTOR_2);

        chooser = new SendableChooser();
        chooser.addDefault("Touch Defense Auto", touchAuto);
        chooser.addObject("Low Bar Auto", lowbarAuto);
        SmartDashboard.putData("Auto choices", chooser);

    }

  
    @Override
    public void teleopInit() {
        leds.activateTeleop();
        arm.calibrate(false);
        shooter.stop();
        arm.pickupAllStop();
        driveAuto.setPIDstate(false);
        drive.setPIDstate(true);
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        drive.set(keyMap.getLeftAxis(), keyMap.getRightAxis());

        if (keyMap.getReverseDriveButton()) {
            keyMap.toggleReverseDrive();
        }
        if (keyMap.getReduceSpeedButton()) {
            keyMap.toggleReduceSpeed();
        }
        if (keyMap.getOverrideDrivePIDButton()) {
            drive.disablePID();
        }

        if (keyMap.getFeedInButton()) {
            arm.gotoPickupPosition();
            arm.feedIn();
            arm.gotoPickupPosition();
        }
        if (keyMap.getFeedOutButton()) {
            arm.feedOut();
        }
        if (keyMap.getFeedStopButton()) {
            shooter.stop();
            arm.pickupAllStop();
        }

        arm.move(keyMap.getArmAxis());
        arm.tick();

        if (keyMap.getGotoShootPositionButton()) {
            arm.gotoShootPosition();
        }
        if (keyMap.getOverrideArmPIDButton()) {
            arm.disablePIDController();
        }

        shooter.tick();
        if (keyMap.getFireButton()) {
            shooter.spinUp();
            firing = true;
            if (keyMap.getFireOverrideButton()) {
                arm.dropBallInShooter();
                shooter.stopWithDelay();
                firing = false;
            }
        }
        if (firing && shooter.hasBeenSpunUp()) {
            arm.dropBallInShooter();
            if (shooter.hasFired()) {
                shooter.stop();
                arm.stopRearPickupWheels();
                firing = false;
            }
        }
        if (keyMap.getOverrideShooterPIDButton()) {
            shooter.enableOverrideMode();
        }

        if (keyMap.getSingleControllerToggleButton()) {
            keyMap.toggleSingleControllerMode();
        }
    }

    
    @Override
    public void autonomousInit() {
       // leds.activateAutonomous();
       // arm.calibrate(true);
    	driveAuto.setPIDstate(true);
        driveAuto.resetEncoders();

        autoSelected = (String) chooser.getSelected();
		SmartDashboard.putString("Auto selected: ", autoSelected);
		
    	autoStep = 0;
    }

    @Override
    public void autonomousPeriodic() {
    	
    	SmartDashboard.putNumber("Auto Step: ", autoStep);
    	
    	switch(autoSelected) {
    	
    	case touchAuto:
    		switch (autoStep) {
		    	case 0: 
		    		driveAuto.driveInches(36, .5);
		    		autoStep++;
		    		break;
		    	case 1:
		    		 if (driveAuto.hasArrived()) {
		    	    	  driveAuto.stop();
		    	    	  SmartDashboard.putString("Drive Target: ", "Arrived");
		    	  			autoStep++;
		
		    	      } else {
		    	    	  driveAuto.updateDriveStatus();
		    			  SmartDashboard.putString("Drive Target: ", "Driving");
		    		      }
		        		break;
		    	case 3:
		    		SmartDashboard.putString("Autonomous Status", "Completed");
		    }
    		break;
    		
    	case lowbarAuto:
    		switch (autoStep) {
	    	case 0: 
	    		driveAuto.driveInches(-12, .5); // backup off line
	    		autoStep++;
	    		break;
	    	case 1:
	    		if (driveAuto.hasArrived()) {
	    	    	  //driveAuto.stop();
	    	    	  SmartDashboard.putString("Auto Step Completed: ", "Drive back 12");
	    	    	  autoStep++;
	    	    } 
	        	break;
	    	case 2:
	       		driveAuto.turnDegrees(90, .6); // turn towards low bar
	    		autoStep++;
	    		break;
	    	case 3:
	    		if (driveAuto.hasArrived()) {
	    			//driveAuto.stop();
	    			SmartDashboard.putString("Auto Step Completed: ", "Turn 90 to face wall");
	    	  		autoStep++;
	    	    } 
	        	break;
	    	case 4:
	       		driveAuto.driveInches(36, .5); // drive towards wall by low bar
	    		autoStep++;
	    		break;
	    	case 5:
	    		if (driveAuto.hasArrived()) {
	    			//driveAuto.stop();
	    			SmartDashboard.putString("Auto Step Completed: ", "Drive towards wall");
	    	  		autoStep++;
	    	    } 
	        	break;
	    	case 6:
	       		driveAuto.turnDegrees(90, .6); // turn to face the low bar
	    		autoStep++;
	    		break;
	    	case 7:
	    		if (driveAuto.hasArrived()) {
	    			//driveAuto.stop();
	    			SmartDashboard.putString("Auto Step Completed: ", "Turn 90 to face low bar");
	    	  		autoStep++;
	    	    } 
	        	break;
	    	case 8:
	       		driveAuto.driveInches(60, .5); // drive through low bar
	    		autoStep++;
	    		break;
	    	case 9:
	    		if (driveAuto.hasArrived()) {
	    			//driveAuto.stop();
	    			SmartDashboard.putString("Auto Step Completed: ", "Drive through low bar");
	    	  		autoStep++;
	    	    } 
	        	break;
	    	case 10:
	       		driveAuto.turnDegrees(45, .6); // turn to face the goal
	    		autoStep++;
	    		break;
	    	case 11:
	    		if (driveAuto.hasArrived()) {
	    			//driveAuto.stop();
	    			SmartDashboard.putString("Auto Step Completed: ", "Turn 45 to face target");
	    	  		autoStep++;
	    	    } 
	        	break;
	    	case 12:
	    		SmartDashboard.putString("Autonomous Status", "Completed");
    		}
		break;   		
    	}
     
	}

 
    @Override
    public void testInit() {
        pdp = new PowerDistributionPanel();
    	testManager.reset();
    	
//        testStage = 0;
//        testTimer = System.currentTimeMillis() + 1000;
//        leds.activateTest();
//        Logger.getInstance().log("test start");
    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {
        arm.tick();
        testManager.step();
//        switch (testStage) {
//            case 0: // Drive Left
//                drive.set(0.5, 0);
//                if (drive.leftEncoderHasError()) {
//                    leds.setColor(RobotLEDs.Color.RED, 2);
//                    hasError = true;
//                }
//                if (System.currentTimeMillis() > testTimer) {
//                    testStage++;
//                    testTimer = System.currentTimeMillis() + 1000;
//                    leds.setColor(RobotLEDs.Color.YELLOW, 2);
//                }
//                break;
//            case 1: // Drive Right
//                drive.set(0, 0.5);
//                if (drive.rightEncoderHasError()) {
//                    leds.setColor(RobotLEDs.Color.RED, 2);
//                    hasError = true;
//                }
//                if (System.currentTimeMillis() > testTimer) {
//                    testStage++;
//                    testTimer = System.currentTimeMillis() + 12000;
//                    leds.setColor(RobotLEDs.Color.YELLOW, 2);
//                    arm.calibrate(true);
//                    drive.set(0, 0);
//                }
//                break;
//            case 2: // Arm
//                if (arm.isCalibrated()) {
//                    arm.gotoRestingPosition();
//
//                    if (arm.passedLimitSwitchTest()) {
//                        testStage++;
//                        testStage++;
//                        testTimer = System.currentTimeMillis() + 2000;
//                        leds.setColor(RobotLEDs.Color.YELLOW, 2);
//                        Logger.getInstance().log("2 passed");
//                    }
//                } else if (System.currentTimeMillis() > testTimer) {
//                    testStage++;
//                    hasError = true;
//                    testTimer = System.currentTimeMillis() + 1500;
//                }
//                break;
//            case 3:
//                leds.setColor(RobotLEDs.Color.RED, 2);
//                hasError = true;
//                if (System.currentTimeMillis() > testTimer) {
//                    testStage++;
//                    testTimer = System.currentTimeMillis() + 2000;
//                }
//                break;
//            case 4: // Shooter 
//                shooter.spinUp();
//                if (shooter.isSpunUp()) {
//                    shooter.stop();
//                    testStage++;
//                    testStage++;
//                    testTimer = System.currentTimeMillis() + 1500;
//                    leds.setColor(RobotLEDs.Color.YELLOW, 2);
//                    Logger.getInstance().log("3 passed");
//                } else if (System.currentTimeMillis() > testTimer) {
//                    testTimer = System.currentTimeMillis() + 1500;
//                    hasError = true;
//                    testStage++;
//                }
//                break;
//            case 5:
//                leds.setColor(RobotLEDs.Color.RED, 2);
//                if (System.currentTimeMillis() > testTimer) {
//                    testTimer = System.currentTimeMillis() + 1500;
//                    leds.setColor(RobotLEDs.Color.YELLOW, 2);
//                }
//                break;
//            case 6: // Pickup Front (Semi-Manual)
//                arm.feedIn();
//                if (pdp.getCurrent(Wiring.PICKUP_FRONT_PDP) > 2) {
//                    testStage++;
//                    testStage++;
//                    testTimer = System.currentTimeMillis() + 1500;
//                    leds.setColor(RobotLEDs.Color.YELLOW, 2);
//                    arm.pickupAllStop();
//                    Logger.getInstance().log("4 passed");
//                } else if (System.currentTimeMillis() > testTimer) {
//                    testTimer = System.currentTimeMillis() + 1500;
//                    hasError = true;
//                    testStage++;
//                }
//                break;
//            case 7:
//                leds.setColor(RobotLEDs.Color.RED, 2);
//                if (System.currentTimeMillis() > testTimer) {
//                    testTimer = System.currentTimeMillis() + 1500;
//                    leds.setColor(RobotLEDs.Color.YELLOW, 2);
//                    arm.pickupAllStop();
//                }
//                break;
//            case 8: // Pickup Rear (Semi-Manual)
//                arm.dropBallInShooter();
//                if (pdp.getCurrent(Wiring.PICKUP_REAR_PDP) > 2) {
//                    testStage++;
//                    testStage++;
//                    testTimer = System.currentTimeMillis() + 1500;
//                    leds.setColor(RobotLEDs.Color.YELLOW, 2);
//                    arm.stopRearPickupWheels();
//                    Logger.getInstance().log("5 passed");
//                } else if (System.currentTimeMillis() > testTimer) {
//                    testTimer = System.currentTimeMillis() + 1500;
//                    hasError = true;
//                }
//                break;
//            case 9:
//                leds.setColor(RobotLEDs.Color.RED, 2);
//                if (System.currentTimeMillis() > testTimer) {
//                    testStage++;
//                    testTimer = System.currentTimeMillis() + 1500;
//                    leds.setColor(RobotLEDs.Color.YELLOW, 2);
//                    arm.stopRearPickupWheels();
//                }
//                break;
//            case 10: // Done
//                if (hasError) {
//                    leds.setColor(RobotLEDs.Color.RED, 2);
//                } else {
//                    leds.setColor(RobotLEDs.Color.GREEN, 1);
//                }
//                break;
//        }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

    }
}
