package com.coderedrobotics.vizzini;

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
    final String testAuto = "Test Auto";
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
        driveAuto = new DriveAuto(drive.getLeftEncoderObject(), drive.getRightEncoderObject(), drive.getLeftPWM(), drive.getRightPWM());
        leds = new RobotLEDs(Wiring.RED_AND_GREEN_LEDS, Wiring.BLUE_LEDS);
        shooter = new Shooter(Wiring.SHOOTER_MOTOR_1, Wiring.SHOOTER_MOTOR_2, Wiring.SHOOTER_LIGHT);

        chooser = new SendableChooser();
        chooser.addDefault("Touch Defense Auto", touchAuto);
        chooser.addObject("Low Bar Auto", lowbarAuto);
        chooser.addObject("Test Auto", testAuto);
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
        driveAuto.setPIDstate(true);
        driveAuto.resetEncoders();

        autoSelected = (String) chooser.getSelected();
        SmartDashboard.putString("Auto selected: ", autoSelected);

        autoStep = 0;
    }

    @Override
    public void autonomousPeriodic() {

        SmartDashboard.putNumber("Auto Step: ", autoStep);

        switch (autoSelected) {
            case testAuto:
                switch (autoStep) {
                    case 0:
                        driveAuto.turnDegrees(-180, .6);
                        autoStep++;
                        break;
                    case 1:
                        if (driveAuto.hasArrived()) {
                            //driveAuto.stop();
                            SmartDashboard.putString("Drive Target: ", "Arrived");
                            autoStep++;

                        } else {
                            driveAuto.updateDriveStatus();
                            SmartDashboard.putString("Drive Target: ", "Driving");
                        }
                        break;
                    case 2:
                        driveAuto.turnDegrees(180, .6);
                        autoStep++;
                        break;
                    case 3:
                        driveAuto.updateDriveStatus();
                        SmartDashboard.putString("Drive Target: ", "Driving");
                        break;
                }

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
