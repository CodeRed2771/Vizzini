package com.coderedrobotics.vizzini.statics;

import com.coderedrobotics.libs.HID.HID;
import com.coderedrobotics.libs.HID.LogitechF310;

/**
 *
 * @author michael
 */
public class SummerKeyMap extends KeyMap {

    // GAMEPADS
    private final HID gp1 = new HID(0);
    private final HID gp2 = new HID(1);
    private final int gamepad1 = 0;
    private final int gamepad2 = 1;

    // MANAGEMENT BOOLEANS
    private boolean reverseDrive = false;
    private boolean singleControllerMode = false;
    private boolean reduceSpeed = false;

    // CONTROLLER 0
    private final HID.Button reduceSpeedButton = LogitechF310.BUMPER_RIGHT;
    private final HID.Button singleControllerModeButton = LogitechF310.STICK_RIGHT;
    private final HID.Axis driveLeftAxis = LogitechF310.STICK_LEFT_Y;
    private final HID.Axis driveRightAxis = LogitechF310.STICK_RIGHT_Y;
    private final HID.Button fireButton = LogitechF310.TRIGGER_RIGHT;
    private final HID.Button feedInButton = LogitechF310.A;
    private final HID.Button feedOutButton = LogitechF310.B;
    private final HID.Button feedStopButton = LogitechF310.X;
    private final HID.Button gotoShooterPositionButton = LogitechF310.Y;
    private final HID.Axis armAxis = LogitechF310.DPAD_Y;
    
    private final HID.Button reverseDriveButton = HID.UNMAPPED;
    private final HID.Button fireOverrideButton = HID.UNMAPPED;
    private final HID.Button cancelFireButton = HID.UNMAPPED;
    private final HID.Button overrideArmPIDButton = HID.UNMAPPED;
    private final HID.Button overrideDrivePIDButton = HID.UNMAPPED;
    private final HID.Button overrideShooterPIDButton = HID.UNMAPPED;
    private final HID.Button shooterLightButton = HID.UNMAPPED;
    private final HID.Button portcullisButton = HID.UNMAPPED;
    private final HID.Axis tapeMeasureAxis = HID.UNMAPPED_AXIS;
    private final HID.Button liftInButton = HID.UNMAPPED;
    private final HID.Button liftOutButton = HID.UNMAPPED;
    private final HID.Button shooterSpeedModifierStraightOuterWorks = HID.UNMAPPED;
    private final HID.Button shooterSpeedModifierLowBar = HID.UNMAPPED;
    private final HID.Button dropBallInShooterNoFire = HID.UNMAPPED;
    
    // BUTTON STATES
    private final HID.ButtonState reverseDriveButtonState = HID.newButtonState();
    private final HID.ButtonState singleControllerModeState = HID.newButtonState();
    private final HID.ButtonState reduceSpeedButtonState = HID.newButtonState();
    private final HID.ButtonState shooterLightButtonState1 = HID.newButtonState();
    private final HID.ButtonState shooterLightButtonState2 = HID.newButtonState();

    public SummerKeyMap() {

    }

    private HID getHID(int gamepad) {
        return gp1;
    }

    public boolean getReverseDriveButton() {
        return getHID(gamepad1).buttonPressed(reverseDriveButton, reverseDriveButtonState);
    }

    public void toggleReverseDrive() {
        reverseDrive = !reverseDrive;
    }

    public boolean getReverseDrive() {
        return reverseDrive;
    }

    public boolean getReduceSpeedButton() {
        return getHID(gamepad1).buttonPressed(reduceSpeedButton, reduceSpeedButtonState);
    }

    public void toggleReduceSpeed() {
        reduceSpeed = !reduceSpeed;
    }

    public boolean getReduceSpeed() {
        return reduceSpeed;
    }

    public void setSingleControllerMode(boolean state) {
        singleControllerMode = state;
    }

    public boolean getSingleControllerMode() {
        return singleControllerMode;
    }

    public void toggleSingleControllerMode() {
        singleControllerMode = !singleControllerMode;
    }

    public boolean getSingleControllerToggleButton() {
        return getHID(gamepad1).buttonPressed(singleControllerModeButton, singleControllerModeState);
    }

    public boolean getFireButton() {
        return getHID(gamepad1).button(fireButton);
    }

    public boolean getFireOverrideButton() {
        return getHID(gamepad1).button(fireOverrideButton);
    }

    public double getLeftAxis() {
        return (reverseDrive ? -(getHID(gamepad1).axis(driveRightAxis)) : (getHID(gamepad1).axis(driveLeftAxis)))
                * (reduceSpeed ? Calibration.DRIVE_TRAIN_REDUCTION_FACTOR : 1);
    }

    public double getRightAxis() {
        return (reverseDrive ? -(getHID(gamepad1).axis(driveLeftAxis)) : (getHID(gamepad1).axis(driveRightAxis)))
                * (reduceSpeed ? Calibration.DRIVE_TRAIN_REDUCTION_FACTOR : 1);
    }

    public boolean getDriverCancelFireButton() {
        return getHID(gamepad1).button(cancelFireButton);
    }

    public double getArmAxis() {
        return -getHID(gamepad2).axis(armAxis);
    }

    public boolean getFeedInButton() {
        return getHID(gamepad2).button(feedInButton);
    }

    public boolean getFeedOutButton() {
        return getHID(gamepad2).button(feedOutButton);
    }

    public boolean getFeedStopButton() {
        return getHID(gamepad2).button(feedStopButton);
    }

    public boolean getGotoShootPositionButton() {
        return getHID(gamepad2).button(gotoShooterPositionButton);
    }

    public boolean getOverrideArmPIDButton() {
        return getHID(gamepad2).button(overrideArmPIDButton);
    }

    public boolean getOverrideDrivePIDButton() {
        return getHID(gamepad2).button(overrideDrivePIDButton);
    }

    public boolean getOverrideShooterPIDButton() {
        return getHID(gamepad2).button(overrideShooterPIDButton);
    }

    public boolean getShooterLightToggleButton() {
        return getHID(gamepad1).buttonPressed(shooterLightButton, shooterLightButtonState1) || getHID(gamepad2).buttonPressed(shooterLightButton, shooterLightButtonState2);
    }

    public boolean getPortcullisButton() {
        return getHID(gamepad2).button(portcullisButton);
    }

    public double getTapeMeasureAxis() {
        return getHID(gamepad2).axis(tapeMeasureAxis);
    }

    public boolean getLiftInButton() {
        return getHID(gamepad2).button(liftInButton);
    }

    public boolean getLiftOutButton() {
        return getHID(gamepad2).button(liftOutButton);
    }

    public boolean getShooterSpeedModifierStraightOuterWorks() {
        return getHID(gamepad2).button(shooterSpeedModifierStraightOuterWorks);
    }

    public boolean getShooterSpeedModifierLowBar() {
        return getHID(gamepad2).button(shooterSpeedModifierLowBar);
    }
    
    public boolean getDropBallInShooterNoFire() {
        return getHID(gamepad2).button(dropBallInShooterNoFire);
    }
}
