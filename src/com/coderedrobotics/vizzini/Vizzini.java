package com.coderedrobotics.vizzini;

import com.coderedrobotics.libs.RobotLEDs;
import com.coderedrobotics.libs.PWMSplitter2X;
import com.coderedrobotics.libs.TankDrive;
import edu.wpi.first.wpilibj.IterativeRobot;
import com.coderedrobotics.vizzini.statics.KeyMap;
import com.coderedrobotics.vizzini.statics.Wiring;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Vizzini extends IterativeRobot {

    // WE WILL NEED A PLACE TRACKER.  
    // ALSO ANOTHER THING TO CONSIDER IS THAT AUTONOMOUS SELECTION IS GOING
    // TO BE VERY COMPLEX, WE WILL NEED TO WORK ON THAT.
    
    KeyMap keyMap;
    Arm arm;
    Shooter shooter;
    Drive drive;
    RobotLEDs leds;
    
    boolean firing = false;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        keyMap = new KeyMap();
        arm = new Arm(Wiring.ARM_MOTOR, Wiring.PICKUP_FRONT_MOTOR, Wiring.PICKUP_REAR_MOTOR);
        drive = new Drive();
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
        leds.activateAutonomous();
        arm.calibrate(true);
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
    	arm.tick();
    }

    @Override
    public void teleopInit() {
        leds.activateTeleop();
        arm.calibrate(false);
        shooter.stop();
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
    public void testInit() {

    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {

    }
    
    @Override
    public void disabledInit() {
        
    }
    
    @Override
    public void disabledPeriodic() {
        
    }
}
