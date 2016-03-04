package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.Logger;
import com.coderedrobotics.libs.RobotLEDs;
import edu.wpi.first.wpilibj.IterativeRobot;
import com.coderedrobotics.vizzini.statics.KeyMap;
import com.coderedrobotics.vizzini.statics.Wiring;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Vizzini extends IterativeRobot {

    KeyMap keyMap;
    Arm arm;
    Shooter shooter;
    Drive drive;
    DriveAuto driveAuto;
    RobotLEDs leds;

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
        keyMap = new KeyMap();
        arm = new Arm(Wiring.ARM_MOTOR, Wiring.PICKUP_FRONT_MOTOR, Wiring.PICKUP_REAR_MOTOR);
        drive = new Drive();
       	driveAuto = new DriveAuto(drive.getLeftEncoderObject(), drive.getRightEncoderObject(), drive.getLeftPWM(), drive.getRightPWM());
        leds = new RobotLEDs(Wiring.RED_AND_GREEN_LEDS, Wiring.BLUE_LEDS);
        shooter = new Shooter(Wiring.SHOOTER_MOTOR_1, Wiring.SHOOTER_MOTOR_2);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
       // leds.activateAutonomous();
       // arm.calibrate(true);
        driveAuto.resetEncoders();
    	
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
    	driveAuto.driveInches(24,.5);
      //  arm.tick();
      driveAuto.showEncoderValues();
      if (driveAuto.hasArrived()) {
    	  driveAuto.stop();
    	  SmartDashboard.putString("Drive Target: ", "Arrived");
      } else {
    	  driveAuto.updateDriveStatus();
		  SmartDashboard.putString("Drive Target: ", "Driving");
	      }
	}

    @Override
    public void teleopInit() {
        leds.activateTeleop();
        arm.calibrate(false);
        shooter.stop();
        arm.pickupAllStop();
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

    PowerDistributionPanel pdp;

    @Override
    public void testInit() {
        testStage = 0;
        testTimer = System.currentTimeMillis() + 1000;
        leds.activateTest();
        pdp = new PowerDistributionPanel();
        Logger.getInstance().log("test start");
    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {
//        arm.tick();
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
//            case 9: // Done
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
