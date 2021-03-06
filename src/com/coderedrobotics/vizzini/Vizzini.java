package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.Logger;
import com.coderedrobotics.libs.RobotLEDs;
import com.coderedrobotics.libs.Timer;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.IterativeRobot;

import com.coderedrobotics.vizzini.statics.Calibration;
import com.coderedrobotics.vizzini.statics.KeyMap;
import com.coderedrobotics.vizzini.statics.SummerKeyMap;
import com.coderedrobotics.vizzini.statics.Wiring;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//
// VIZZINI - 2016 - STRONGHOLD GAME
//
public class Vizzini extends IterativeRobot {

    TestManager testManager;
    KeyMap currentKeyMap;
    KeyMap normalKeyMap;
    SummerKeyMap summerKeyMap;
    Arm arm;
    Shooter shooter;
    Drive drive;
    DriveAuto driveAuto;
    Lift lift;
    RobotLEDs leds;
    PowerDistributionPanel pdp;
    SendableChooser chooser;
    AnalogGyro gyro;
    Timer autoTimer;

    //private MotionProfileTrapezoidal motionProfile;
    //private DriveAutoMP driveAutoMP;
    private double autoStartTime;

    final String lowbarFollowThruAuto = "Low Bar Follow Thru";
    final String lowbarStraightThru = "lowbarStraightThru";
    final String touchAuto = "Touch Defense Auto";
    final String testAutoTurn = "Test Auto Turn";
    final String testIncDrive = "Test Incremental Drive";
    final String chivalAuto = "ChivalDeFrise";
    final String chivalNoShotAuto = "chivalNoShotAuto";
    final String testAutoDefenseFire = "Test AutoDefenseFire";
    final String motionAutoTest = "MotionAutoTest";
    final String testEncoderMeasure = "Test Encoder Measure";

    String autoSelected;

    boolean firing = false;
    private int testStage = 0;
    private long testTimer = 0;
    private boolean hasError = false;
    private double robotPosition = 0;
    private double autoTurnDegrees = 50;

    @Override
    public void robotInit() {
        autoTimer = new Timer();
        
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

        gyro = new AnalogGyro(Wiring.GYRO);
        gyro.initGyro();
        gyro.calibrate();
        summerKeyMap = new SummerKeyMap();
        normalKeyMap = new KeyMap();
        currentKeyMap = summerKeyMap;
        leds = new RobotLEDs(Wiring.RED_AND_GREEN_LEDS, Wiring.BLUE_LEDS);
        arm = new Arm(Wiring.ARM_MOTOR, Wiring.PICKUP_FRONT_MOTOR, Wiring.PICKUP_REAR_MOTOR, leds);
        drive = new Drive();
        driveAuto = new DriveAuto(drive, gyro);
        shooter = new Shooter(Wiring.SHOOTER_MOTOR_1, Wiring.SHOOTER_MOTOR_2, Wiring.SHOOTER_LIGHT);
        lift = new Lift(Wiring.TAPE_MEASURE_MOTOR, Wiring.LIFT_MOTOR);

        chooser = new SendableChooser();
        chooser.addDefault("Low Bar STRAIGHT Thru", lowbarStraightThru);
        chooser.addObject("REACH the Defense", touchAuto);
        chooser.addObject("Low Bar FOLLOW Through", lowbarFollowThruAuto);
        chooser.addObject("CHIVAL De Frise Auto WITH Shot (REVERSE DIRECTION!)", chivalAuto);
        chooser.addObject("CHIVAL De Frise Auto NO SHOT (REVERSE LINEUP)", chivalNoShotAuto);
        chooser.addObject("Test Auto Turn", testAutoTurn);
        chooser.addObject("Test Incremental Drive", testIncDrive);
        chooser.addObject("Test Auto Defense Fire", testAutoDefenseFire);
        chooser.addObject("Test Encoder Measure (push)", testEncoderMeasure);

        SmartDashboard.putData("Auto choices", chooser);
        SmartDashboard.putNumber("Robot Position (FIELD #)", robotPosition);
        SmartDashboard.putNumber("AUTO TURN DEGREES", autoTurnDegrees);

        SmartDashboard.putNumber("ROT P", Calibration.AUTO_GYRO_P);
        SmartDashboard.putNumber("ROT I", Calibration.AUTO_GYRO_I);
        SmartDashboard.putNumber("ROT D", Calibration.AUTO_GYRO_D);

        SmartDashboard.putNumber("AUTO DRIVE P", Calibration.AUTO_DRIVE_P);
        SmartDashboard.putNumber("AUTO DRIVE I", Calibration.AUTO_DRIVE_I); 
        SmartDashboard.putNumber("AUTO DRIVE D", Calibration.AUTO_DRIVE_D);
        
        SmartDashboard.putString("LOG NOTE", "");

    }

    @Override
    public void teleopInit() {
        leds.activateTeleop();
        arm.calibrate(true);
        shooter.stop();
        shooter.lightOff();
        arm.pickupAllStop();
        driveAuto.setPIDstate(false);
        drive.setPIDstate(true);
    }

    @Override
    public void teleopPeriodic() {

        drive.set(currentKeyMap.getLeftAxis(), currentKeyMap.getRightAxis());

        SmartDashboard.putNumber("Left Axis", currentKeyMap.getLeftAxis());
        SmartDashboard.putNumber("Right Axis", currentKeyMap.getRightAxis());

        if (currentKeyMap.getReverseDriveButton()) {
            currentKeyMap.toggleReverseDrive();
        }
        if (currentKeyMap.getReduceSpeedButton()) {
            currentKeyMap.toggleReduceSpeed();
        }
        if (currentKeyMap.getOverrideDrivePIDButton()) {
            drive.disablePID();
        }
        if (currentKeyMap.getSingleControllerToggleButton()) {
            if (currentKeyMap instanceof SummerKeyMap) {
                currentKeyMap = normalKeyMap;
            } else if (currentKeyMap instanceof KeyMap) {
                currentKeyMap = summerKeyMap;
            } 
        }
        
        if (currentKeyMap.getFeedInButton()) {
            arm.gotoPickupPosition();
            arm.feedIn();
            arm.gotoPickupPosition();
        }
        if (currentKeyMap.getFeedOutButton()) {
            arm.feedOut();
        }
        if (currentKeyMap.getFeedStopButton() || currentKeyMap.getDriverCancelFireButton()) {
            shooter.stop();
            firing = false;
            arm.pickupAllStop();
            shooter.closeGate();
        }

        arm.move(currentKeyMap.getArmAxis());
        arm.tick();

        if (currentKeyMap.getGotoShootPositionButton()) {
            arm.gotoShootPosition();
            shooter.spinUp();
            shooter.lightOn();
        }

        if (currentKeyMap.getDropBallInShooterNoFire()) {
            arm.dropBallInShooter();
            arm.feedInNudge();
        }

        if (currentKeyMap.getOverrideArmPIDButton()) {
            arm.disablePIDController();
        }

        if (currentKeyMap.getPortcullisButton()) {
            arm.feedOut();
            arm.gotoPortcullisPosition();
        }

        shooter.tick();

        if (currentKeyMap.getFireButton()) {
            shooter.spinUp();
            shooter.lightOn();
            firing = true;
            if (currentKeyMap.getFireOverrideButton()) {
                arm.dropBallInShooter();
                shooter.openGate();
                shooter.stopWithDelay();
                firing = false;
            }
        }
        
        SmartDashboard.putBoolean("Firing", firing);
        SmartDashboard.putBoolean("Spun Up", shooter.hasBeenSpunUp());
        
        if (firing && shooter.hasBeenSpunUp()) {
            arm.feedInNudge();
            arm.dropBallInShooter();
            shooter.openGate();
            if (shooter.hasFired()) {
                shooter.closeGate();
                shooter.stop();
                shooter.lightOff();
                arm.pickupAllStop();
                firing = false;
            }
        }
        if (currentKeyMap.getShooterSpeedModifierStraightOuterWorks()) {
            shooter.setSpeedStraightOuterWorks();
        } else if (currentKeyMap.getShooterSpeedModifierLowBar()) {
            shooter.setSpeedLowBar();
        } else {
            shooter.setDefaultSpeed();
        }
        if (currentKeyMap.getShooterLightToggleButton()) {
            shooter.toggleLight();
        }
        if (currentKeyMap.getOverrideShooterPIDButton()) {
            shooter.enableOverrideMode();
        }

        if (currentKeyMap.getLiftInButton()) {
            lift.liftIn();
        } else if (currentKeyMap.getLiftOutButton()) {
            lift.liftOut();
        } else {
            lift.stop();
        }
        lift.tapeMeasure(currentKeyMap.getTapeMeasureAxis());

//        if (keyMap.getSingleControllerToggleButton()) { // NOT DURING A LIVE MATCH YOU DINGuS
//            keyMap.toggleSingleControllerMode();
//        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                              AUTONOMOUS INIT
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void autonomousInit() {
    	driveAuto.reset();    	
        leds.activateAutonomous();
        drive.setPIDstate(true);
        driveAuto.setPIDstate(true);

        autoSelected = (String) chooser.getSelected();
        SmartDashboard.putString("Auto selected: ", autoSelected);
        autoTurnDegrees = (int) SmartDashboard.getNumber("AUTO TURN DEGREES");
        SmartDashboard.putNumber("Degree specified", autoTurnDegrees);

        autoTimer.setStage(0);
        autoTimer.resetTimer(100000); // make sure timer doesn't "hit" until we set it later


        driveAuto.rotDrivePID.setPID(SmartDashboard.getNumber("ROT P"), SmartDashboard.getNumber("ROT I"), SmartDashboard.getNumber("ROT D"));
        driveAuto.drivePID.setPID(SmartDashboard.getNumber("AUTO DRIVE P"), SmartDashboard.getNumber("AUTO DRIVE I"), SmartDashboard.getNumber("AUTO DRIVE D"));
        
//      motionProfile = new MotionProfileTrapezoidal();
//    	motionProfile.SetAccel(SmartDashboard.getNumber("MP Accel"));
//    	motionProfile.SetDecel(SmartDashboard.getNumber("MP Decel"));
//    	motionProfile.SetMaxSpeed(SmartDashboard.getNumber("MP MaxSpeed"));
        autoStartTime = System.currentTimeMillis();

    }

    int laststage = -1;
    boolean hasprintedcalibrated;
    boolean lastcalibration;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                              AUTONOMOUS PERIODIC
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void autonomousPeriodic() {

        SmartDashboard.putNumber("Auto Step: ", autoTimer.getStage());

        switch (autoSelected) {

        	case testEncoderMeasure:
        		doTestEncoderMeasure();
        		break;
        		
            case motionAutoTest:
                //doMotionAutoTest();
                break;

            case testAutoDefenseFire:
                doTestAutoDefenseFire(); //simply testing automatic defense auto
                break;

            case testIncDrive:
                doTestDriveAuto();
                break;

            case testAutoTurn:
                doTestTurnAuto();
                break;

            case touchAuto:
                doTouchDefenseAuto();
                break;

            case chivalAuto:
                doChivalAuto(true);
                break;
                
            case chivalNoShotAuto:
            	doChivalAuto(false);
            	break;
            	
            case lowbarStraightThru:
                doLowBarStraightThruAuto();  // main auto
                break;

            case lowbarFollowThruAuto:
                doLowBarFollowThruAuto(); // from second position
                break;

        }

        arm.tick();
        driveAuto.tick(); // tick currently manages power ramping
        driveAuto.showEncoderValues();
        autoTimer.advanceWhenReady();  // this will advance the stage when the timer expires

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //        LOW BAR STRAIGHT THRU AND SHOOT - MAIN AUTO PROGRAM
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void doLowBarStraightThruAuto() {
        //
        //
        // LOW BAR - STRAIGHT THRU - SHOOT - HEAD BACK
        //
        // MAIN Auto Routine
        //
        switch (autoTimer.getStage()) {
            case 0:
                shooter.openGate();
                autoTimer.setTimerAndAdvanceStage(6000);
                driveAuto.driveInches(198, .5);
                arm.calibrate(true);
                break;
            case 1:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }// else if (driveAuto.getDistanceTravelled() > 50 && driveAuto.getDistanceTravelled() < 155) {
                  //  driveAuto.setMaxPowerOutput(.55);
                //} //				if (driveAuto.getDistanceTravelled() > 155) 
                //					driveAuto.setMaxPowerOutput(.25);
                break;
            case 2:
                autoTimer.setTimerAndAdvanceStage(2000);
                arm.calibrate(true);
                break;
            case 3:
                if (arm.isCalibrated()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 4:
                autoTimer.setTimerAndAdvanceStage(2000); // was 4 secs
                arm.gotoShootPosition();
                driveAuto.turnDegrees(autoTurnDegrees, .6);    //   MAIN AUTO TURN FOR THE SHOT
                shooter.spinUp();
                break;
            case 5:
                if (driveAuto.turnCompleted()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 6:
                autoTimer.setTimerAndAdvanceStage(3000);
                driveAuto.driveInches(18, .5); // Go forward 1 foot and a half
                break;
            case 7:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;

            case 8:
                autoTimer.setTimerAndAdvanceStage(3000);
                // driveAuto.stop();
                arm.dropBallInShooter();//Drops the ball in the shooter
                break;
            case 9:
                if (shooter.hasFired()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                //wait for shooter to shoot
                break;
            case 10:
                autoTimer.setTimerAndAdvanceStage(2000);
                shooter.stop();
                arm.pickupAllStop();
                driveAuto.driveInches(-18, .4); // Go backward 1 foot and a half
                break;
            case 11:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 12:
                autoTimer.setTimerAndAdvanceStage(2000);
                driveAuto.turnDegrees(-autoTurnDegrees, .7);
                break;
            case 13:
                if (driveAuto.turnCompleted()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 14:
                autoTimer.setTimerAndAdvanceStage(3000);
                arm.gotoPickupPosition();
                driveAuto.driveInches(-55, .4);
            case 15:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 16:
                //driveAuto.stop();

                //Have a nice day! :)
                break;
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void doLowBarFollowThruAuto() {
        // Follow through the low bar from the second position

        switch (autoTimer.getStage()) {
            case 0:
//			autoTimer.setTimerAndAdvanceStage(1000); // wait a bit before starting to give alliance time to get out of the way
                autoTimer.setTimerAndAdvanceStage(1);
                break;
            case 1:
                // do nothing - the timer will expire and move to the next stage
                break;
            case 2:
                autoTimer.setTimerAndAdvanceStage(2000);
                driveAuto.driveInches(12, .4); // move away from the line
                break;
            case 3:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 4:
                autoTimer.setTimerAndAdvanceStage(3000);
                driveAuto.turnDegrees(-90, .6); // turn towards wall
                break;
            case 5:
                if (driveAuto.turnCompleted()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 6:
                autoTimer.setTimerAndAdvanceStage(3000);
                arm.calibrate(true);
                driveAuto.driveInches(55, .4); // drive towards wall by low bar
                break;
            case 7:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 8:
                autoTimer.setTimerAndAdvanceStage(2000);
                driveAuto.turnDegrees(90, .6); // turn to face the low bar

                break;
            case 9:
                if (driveAuto.turnCompleted()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 10:
                autoTimer.setTimerAndAdvanceStage(4000);
                driveAuto.driveInches(115, .70); // drive through low bar
                break;
            case 11:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 12:
                if (driveAuto.getDistanceTravelled() < 90) { // stuck on lowbar
                	driveAuto.setMaxPowerOutput(.1); // cut the power down cuz we're stalled anyway
                    autoTimer.setStage(99); // jump out of auto
                }
                autoTimer.setTimerAndAdvanceStage(2000);
                driveAuto.turnDegrees(38, .7); // turn to face the tower  (added 9 from first miss on 4/14
                shooter.lightOn();
                arm.gotoRestingPosition();
                //arm.gotoShootPosition();
                shooter.setSpeedLowBar();
                shooter.spinUp();
                break;
            case 13:
                if (driveAuto.turnCompleted()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 14:
                autoTimer.setTimerAndAdvanceStage(2000);
                driveAuto.driveInches(20, .7); 
                break;
            case 15:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 16:
                // no timer on this step cuz it will run out of auto time if it doesn't shoot
                if (shooter.hasBeenSpunUp()) {
                    shooter.openGate();
                  //  arm.dropBallInShooter();//Drops the ball in the shooter
                    autoTimer.stopTimerAndAdvanceStage();
                }

                break;
            case 17:
                if (shooter.hasFired()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                //wait for shooter to shoot
                break;
            case 18:
                autoTimer.setTimerAndAdvanceStage(2000);
                shooter.stop();
                arm.pickupAllStop();
                arm.gotoPickupPosition();
                break;
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void doChivalAuto(boolean takeShot) {
        //
        // CHIVAL DE FREESE
        //
        switch (autoTimer.getStage()) {
            case 0:
                shooter.openGate();
                autoTimer.setTimerAndAdvanceStage(2000);
                arm.calibrate(true);
                break;
            case 1:
                //Waiting for timer to expire
                break;
            case 2:
                autoTimer.setTimerAndAdvanceStage(3000);
                arm.gotoChivalDeFrisePosition();
                driveAuto.driveInches(-50, .35);
                break;
            case 3:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 4:
            	driveAuto.reset(); // reset encoders before we back up a little
            	driveAuto.driveInches(6, .7, .3); // backup a little
                autoTimer.setTimerAndAdvanceStage(1500);   
                arm.gotoPickupPosition();
                break;
            case 5:
                //Waiting for timer to expire
                break;
            case 6:
                autoTimer.setTimerAndAdvanceStage(4000);
                driveAuto.driveInches(-106, .72, .3);
                break;
            case 7:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                } else {
                    if (driveAuto.getDistanceTravelled() > 25) {
                        driveAuto.setMaxPowerOutput(.4);
                    }
                }
                break;
            //STARTING SHOOTING CASES
            case 8:
                if (driveAuto.getDistanceTravelled() < 50) { // stuck on chival
                    autoTimer.setStage(15);
                }
                autoTimer.setTimerAndAdvanceStage(3000);

                robotPosition = SmartDashboard.getNumber("Robot Position (FIELD #)", 0);

                switch ((int) robotPosition) {
                	case 1: // should never be
                		autoTimer.setStage(14); // abort auto
                		break;
                    case 2:
                        driveAuto.turnDegrees(-140, .7);
                        break;
                    case 3:
                        driveAuto.turnDegrees(-175, .7);
                        break;
                    case 4:
                        driveAuto.turnDegrees(170, .7);
                        break;
                    case 5:
                        driveAuto.turnDegrees(150, .7);
                        break;
                }
                break;
            case 9:
                if (driveAuto.turnCompleted()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 10:
                autoTimer.setTimerAndAdvanceStage(3000);
                shooter.lightOn();
                if (takeShot) {
                shooter.spinUp();
                }
                arm.gotoShootPosition();
                driveAuto.driveInches(12, .6);
                break;
            case 11:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 12:
                autoTimer.setTimerAndAdvanceStage(3000);
                if (takeShot) { 
                	arm.dropBallInShooter(); 
                }
                break;
            case 13:
                if (shooter.hasFired()) { //wait for shooter to shoot
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 14:
                autoTimer.setTimerAndAdvanceStage(3000);
                if (takeShot) {
	                shooter.stop();
	                arm.pickupAllStop();
	                arm.gotoPickupPosition();
	            }
                
                break;

        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void doTouchDefenseAuto() {
        switch (autoTimer.getStage()) {
            case 0:
                autoTimer.setTimerAndAdvanceStage(3000);
                driveAuto.driveInches(40, .4);
                break;
            case 1:
                if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                    arm.calibrate(true);

                }
                break;
            case 3:
                SmartDashboard.putString("Autonomous Status", "Completed");
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void doTestAutoDefenseFire() {
        switch (autoTimer.getStage()) {
            case 0:
                shooter.openGate();
                autoTimer.setTimerAndAdvanceStage(3000);
                robotPosition = SmartDashboard.getNumber("Robot Position (From Lowbar)", 0);
                SmartDashboard.putNumber("Robot Pos Read from dash", robotPosition);
                switch ((int) robotPosition) {
                    case 1:
                        driveAuto.turnDegrees(40, .6);
                        break;
                    case 2:
                        driveAuto.turnDegrees(5, .6);
                        break;
                    case 3:
                        driveAuto.turnDegrees(-8, .6);
                        break;
                    case 4:
                        driveAuto.turnDegrees(-30, .6);
                        break;
                }
                break;
            case 1:
                if (driveAuto.turnCompleted()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 2:
                // 				autoTimer.setTimerAndAdvanceStage(2500);
                // 				shooter.spinUp();
                // 				arm.gotoShootPosition();
                break;
            case 3:
                break;
            case 4:
                autoTimer.setTimerAndAdvanceStage(2000);
                arm.dropBallInShooter();
                break;
            case 5:
                if (shooter.hasFired()) {
                    autoTimer.stopTimerAndAdvanceStage();
                }
                //wait for shooter to shoot
                break;
            case 6:
                autoTimer.setTimerAndAdvanceStage(3000);
                shooter.stop();
                arm.pickupAllStop();
                arm.gotoPickupPosition();
                break;

        }

    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void doTestTurnAuto() {
        switch (autoTimer.getStage()) {

            case 0:
                autoTimer.setTimerAndAdvanceStage(4000);
                driveAuto.turnDegrees(45, .7);
                break;

            case 1:
                SmartDashboard.putBoolean("TURN COMPLETED", driveAuto.turnCompleted());
                break;

            case 2:
                autoTimer.setTimerAndAdvanceStage(4000);
                driveAuto.turnDegrees(45, .7);
                break;

            case 3:
                SmartDashboard.putBoolean("TURN COMPLETED", driveAuto.turnCompleted());
                break;

            case 4:
                autoTimer.setTimerAndAdvanceStage(4000);
                driveAuto.turnDegrees(45, .7);
                break;

            case 5:
                SmartDashboard.putBoolean("TURN COMPLETED", driveAuto.turnCompleted());
                break;
            case 6:
                autoTimer.setTimerAndAdvanceStage(4000);
                driveAuto.turnDegrees(45, .7);
                break;

            case 7:
                SmartDashboard.putBoolean("TURN COMPLETED", driveAuto.turnCompleted());
                break;
        }

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    private void doTestEncoderMeasure() {
    	// basically resets the encoders and then lets you push the robot
    	// and watch the encoder values change on the SmartDashboard
    	switch (autoTimer.getStage()) {

        case 0:
            autoTimer.setTimerAndAdvanceStage(6000);
            driveAuto.reset();
            break;
        case 1:
            autoTimer.stopTimerAndAdvanceStage();
            break;
        case 2:
            break;


    	}
		
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void doTestDriveAuto() {
    	
    	int distance = 4;  
    	
        switch (autoTimer.getStage()) {

            case 0:
                autoTimer.setTimerAndAdvanceStage(8000);
                arm.calibrate(true);
                driveAuto.driveInches(distance, .4);
                break;
            case 1:
                if (driveAuto.hasArrived()) {
                   arm.gotoRestingPosition();
                   autoTimer.stopTimerAndAdvanceStage();
                }
                break;
            case 2:
            	autoTimer.setTimerAndAdvanceStage(5000);
                break;
            case 3: 
            	break;
            case 4:
            	autoTimer.setTimerAndAdvanceStage(8000);
            	driveAuto.driveInches(-distance, .4);
            	break;
            case 5:
            	if (driveAuto.hasArrived()) {
                    autoTimer.stopTimerAndAdvanceStage();
                 }
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void doMotionAutoTest() {
        double elapsedTime = System.currentTimeMillis() - autoStartTime;

//    	switch (autoTimer.getStage()) {
//		case 0:
//			motionProfile.SetDistance(convertToTicks(120));
//			motionProfile.CalcParams();
//			autoTimer.nextStage();
//			elapsedTime = System.currentTimeMillis() - autoStartTime;
//			// break;  (intentionally left out)
//		case 1:
//			double newPosition = motionProfile.Position(elapsedTime);
//			driveAutoMP.setPosition(newPosition, newPosition);
//			break;
//		}
//		
//		SmartDashboard.putNumber("MP Accel Time", motionProfile.GetAccelTime());
//		SmartDashboard.putNumber("MP Decel Time", motionProfile.GetDecelTime());
//		
//		driveAutoMP.showEncoderValues();
    }

    @Override
    public void testInit() {
        driveAuto.setPIDstate(false);
        drive.setPIDstate(true);
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

    private int convertToTicks(int inches) {
        return (int) (inches * Calibration.DRIVE_DISTANCE_TICKS_PER_INCH);
    }

}
