package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.Logger;
import com.coderedrobotics.libs.RobotLEDs;
import com.coderedrobotics.libs.Timer;

import edu.wpi.first.wpilibj.DriverStation;
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
    Timer autoTimer;
    final String lowbarAuto = "Low Bar";
    final String lowbarStraightThru = "lowbarStraightThru";
    final String touchAuto = "Touch Defense Auto";
    final String testAuto = "Test Auto";
    final String testAutoTurn = "Test Auto Turn";
    String autoSelected;

    int autoStep = 0;

    boolean firing = false;

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
        }, () -> {
            drive.set(0, 0);
        }, 1000, TestResult.SUCESS, "drive");
        testManager.addStage(() -> {
            arm.calibrate(true);
            return arm.isCalibrated() ? TestResult.SUCESS : TestResult.INCONCLUSIVE;
        }, () -> {
        }, 10000, TestResult.FAILURE, "arm calibration");
        testManager.addStage(() -> {
            arm.gotoRestingPosition();
            return arm.passedLimitSwitchTest() ? TestResult.SUCESS : TestResult.INCONCLUSIVE;
        }, () -> {
        }, 3000, TestResult.FAILURE, "arm");
        testManager.addStage(() -> {
            shooter.spinUp();
            return shooter.isSpunUp() ? TestResult.SUCESS : TestResult.INCONCLUSIVE;
        }, () -> {
            shooter.stop();
        }, 5000, TestResult.FAILURE, "shooter");
        testManager.addStage(() -> {
            arm.feedIn();
            return pdp.getCurrent(Wiring.PICKUP_FRONT_PDP) > 2 ? TestResult.SUCESS : TestResult.INCONCLUSIVE;
        }, () -> {
            arm.pickupAllStop();
        }, 1000, TestResult.FAILURE, "pickup front");
        testManager.addStage(() -> {
            arm.dropBallInShooter();
            return pdp.getCurrent(Wiring.PICKUP_REAR_PDP) > 2 ? TestResult.SUCESS : TestResult.INCONCLUSIVE;
        }, () -> {
            arm.pickupAllStop();
        }, 1000, TestResult.FAILURE, "pickup rear");

        keyMap = new KeyMap();
        arm = new Arm(Wiring.ARM_MOTOR, Wiring.PICKUP_FRONT_MOTOR, Wiring.PICKUP_REAR_MOTOR);
        drive = new Drive();
        driveAuto = new DriveAuto(drive);
        leds = new RobotLEDs(Wiring.RED_AND_GREEN_LEDS, Wiring.BLUE_LEDS);
        shooter = new Shooter(Wiring.SHOOTER_MOTOR_1, Wiring.SHOOTER_MOTOR_2, Wiring.SHOOTER_LIGHT);

        chooser = new SendableChooser();
        chooser.addObject("Touch Defense Auto", touchAuto);
        chooser.addObject("Low Bar One Away", lowbarAuto);
        chooser.addDefault("Low Bar Straight Thru", lowbarStraightThru);
        chooser.addObject("Test Auto Drive 10'", testAuto);
        chooser.addObject("Test Auto Turn 180'", testAutoTurn);
        SmartDashboard.putData("Auto choices", chooser);

    }

    @Override
    public void teleopInit() {
        leds.activateTeleop();
        arm.calibrate(true);
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
        if (keyMap.getFeedStopButton() || keyMap.getDriverCancelFireButton()) {
            shooter.stop();
            firing = false;
            arm.pickupAllStop();
        }

        arm.move(keyMap.getArmAxis());
        arm.tick();

        if (keyMap.getGotoShootPositionButton()) {
            arm.gotoShootPosition();
            shooter.spinUp();
            shooter.lightOn();
        }
        if (keyMap.getOverrideArmPIDButton()) {
            arm.disablePIDController();
        }

        shooter.tick();
        if (keyMap.getFireButton()) {
            shooter.spinUp();
            shooter.lightOn();
            firing = true;
            if (keyMap.getFireOverrideButton()) {
                arm.dropBallInShooter();
                shooter.stopWithDelay();
                firing = false;
            }
        }
        if (firing && shooter.hasBeenSpunUp()) {
            arm.dropBallInShooter();
            arm.feedInNudge();
            if (shooter.hasFired()) {
                shooter.stop();
                shooter.lightOff();
                arm.pickupAllStop();
                firing = false;
            }
        }
        if (keyMap.getOverrideShooterPIDButton()) {
            shooter.enableOverrideMode();
        }
        if (keyMap.getShooterLightToggleButton()) {
            shooter.toggleLight();
        }

        if (keyMap.getSingleControllerToggleButton()) {
            keyMap.toggleSingleControllerMode();
        }
    }

    @Override
    public void autonomousInit() {
        //leds.activateAutonomous();
        drive.setPIDstate(true);
        driveAuto.setPIDstate(true);

        autoSelected = (String) chooser.getSelected();
        SmartDashboard.putString("Auto selected: ", autoSelected);

        autoTimer.setStage(0);
        autoTimer.resetTimer(100000); // make sure timer doesn't "hit" until we set it later

        Logger.getInstance().log("start auto");
    }

    int laststage = -1;
    boolean hasprintedcalibrated;
    boolean lastcalibration;

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putNumber("Auto Step: ", autoTimer.getStage());
//    	if (autoTimer.getStage() != laststage || !hasprintedcalibrated || arm.isCalibrated() != lastcalibration) {
//    		Logger.getInstance().log("Calibrated : " + arm.isCalibrated() + "\tStage: " + autoTimer.getStage());
//    		laststage = autoTimer.getStage();
//    		hasprintedcalibrated = true;
//    		lastcalibration = arm.isCalibrated();
//    	}
        arm.tick();

        switch (autoSelected) {

            case testAuto:
                switch (autoTimer.getStage()) {

                    case 0:
                        autoTimer.setTimerAndAdvanceStage(3000);
                        driveAuto.driveInches(120, .7);
                        break;

                    case 1:
                        //	 if (driveAuto.hasArrived()) {
                        //    	  //driveAuto.stop();
                        //    	  	SmartDashboard.putString("Drive Target: ", "Arrived");
                        //  			autoTimer.stopTimerAndAdvanceStage();
                        // 	      } else {
                        //    	  driveAuto.updateDriveStatus();
                        //		  SmartDashboard.putString("Drive Target: ", "Driving");
                        //	      }
                        break;

                }

                break;

            case testAutoTurn:
                switch (autoTimer.getStage()) {

                    case 0:
                        autoTimer.setTimerAndAdvanceStage(3000);
                        driveAuto.turnDegrees(180, .7);
                        break;

                    case 1:
                        //	 if (driveAuto.hasArrived()) {
                        //    	  //driveAuto.stop();
                        //    	  	SmartDashboard.putString("Drive Target: ", "Arrived");
                        //  			autoTimer.stopTimerAndAdvanceStage();
                        // 	      } else {
                        //    	  driveAuto.updateDriveStatus();
                        //		  SmartDashboard.putString("Drive Target: ", "Driving");
                        //	      }
                        break;

                }

                break;

            case touchAuto:
                switch (autoTimer.getStage()) {
                    case 0:
                        autoTimer.setTimerAndAdvanceStage(3000);
                        driveAuto.driveInches(36, .5);
                        break;
                    case 1:
                        if (driveAuto.hasArrived()) {
                            autoTimer.stopTimerAndAdvanceStage();
                            SmartDashboard.putString("Drive Target: ", "Arrived");

                        } else {

                            SmartDashboard.putString("Drive Target: ", "Driving");
                        }
                        break;
                    case 3:
                        SmartDashboard.putString("Autonomous Status", "Completed");
                }

                break;

            case lowbarStraightThru:
                switch (autoTimer.getStage()) {
                    case 0:
                        autoTimer.setTimerAndAdvanceStage(2000);
                        driveAuto.driveInches(12, .3); // Go forward 1 foot
                        break;
                    case 1:
                        if (driveAuto.hasArrived()) {
                            autoTimer.stopTimerAndAdvanceStage();
                        }
                        break;
                    case 2:
                        autoTimer.setTimerAndAdvanceStage(15000);  //use all of auto to calibrate cuz if it doesn't calibrate 
                        arm.calibrate(true);
                        break;
                    case 3:
                        if (arm.isCalibrated()) {
                            autoTimer.stopTimerAndAdvanceStage();
                        }
                        break;
                    case 4:
                        autoTimer.setTimerAndAdvanceStage(5000);
                        driveAuto.driveInches(210, .3);
                        break;
                    case 5:
                        if (driveAuto.hasArrived()) {
                            autoTimer.stopTimerAndAdvanceStage();
                        }
                        break;
                    case 6:
                        autoTimer.setTimerAndAdvanceStage(4000);
                        driveAuto.turnDegrees(60, .6);
                        arm.calibrate(true);  // force arm recalibrate
                        break;
                    case 7:
                        if (driveAuto.hasArrived()) {
                            autoTimer.stopTimerAndAdvanceStage();
                        }
                        break;
                    case 8:
                        autoTimer.setTimerAndAdvanceStage(3000);
                        driveAuto.stop();
                        arm.gotoShootPosition();
                        shooter.spinUp();
                        break;
                    case 9:
                        // Wait for shooter to spin up
                        break;
                    case 10:
                        autoTimer.setTimerAndAdvanceStage(3000);
                        arm.dropBallInShooter();//Drops the ball in the shooter
                        break;
                    case 11:
                        //wait for shooter to shoot
                        break;
                    case 12:
                        autoTimer.setTimerAndAdvanceStage(3000);
                        shooter.stop();
                        arm.gotoPickupPosition();
                        arm.pickupAllStop();
                        break;
                    case 13:
                        //Have a nice day! :)
                        break;
                }
                break;

            case lowbarAuto:
                switch (autoTimer.getStage()) {
                    case 0:
                        autoTimer.setTimerAndAdvanceStage(2000);
                        driveAuto.driveInches(12, .5); // move away from the line
                        break;
                    case 1:
                        if (driveAuto.hasArrived()) {
                            autoTimer.stopTimerAndAdvanceStage();
                        }
                        break;
                    case 2:
                        autoTimer.setTimerAndAdvanceStage(2000);
                        driveAuto.turnDegrees(-90, .6); // turn towards low bar
                        break;
                    case 3:
                        if (driveAuto.hasArrived()) {
                            autoTimer.stopTimerAndAdvanceStage();
                        }
                        break;
                    case 4:
                        autoTimer.setTimerAndAdvanceStage(2000);
                        driveAuto.driveInches(36, .5); // drive towards wall by low bar
                        break;
                    case 5:
                        if (driveAuto.hasArrived()) {
                            autoTimer.stopTimerAndAdvanceStage();
                        }
                        break;
                    case 6:
                        autoTimer.setTimerAndAdvanceStage(2000);
                        driveAuto.turnDegrees(90, .6); // turn to face the low bar

                        break;
                    case 7:
                        if (driveAuto.hasArrived()) {
                            autoTimer.stopTimerAndAdvanceStage();
                        }
                        break;
                    case 8:
                        autoTimer.setTimerAndAdvanceStage(5000);
                        driveAuto.driveInches(90, .5); // drive through low bar
                        break;
                    case 9:
                        if (driveAuto.hasArrived()) {
                            autoTimer.stopTimerAndAdvanceStage();
                        }
                        break;
                    case 10:
                        autoTimer.setTimerAndAdvanceStage(2000);
                        driveAuto.turnDegrees(45, .6); // turn to face the goal
                        break;
                    case 11:
                        if (driveAuto.hasArrived()) {
                            autoTimer.stopTimerAndAdvanceStage();
                        }
                        break;
                    case 12:
                        driveAuto.stop();
                }
                break;
        }

        driveAuto.showEncoderValues();
        autoTimer.advanceWhenReady();
    }

    @Override
    public void testInit() {
        driveAuto.setPIDstate(false);
        drive.setPIDstate(true);
        pdp = new PowerDistributionPanel();
        testManager.reset();
    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {
        arm.tick();
        testManager.step();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

    }
}
