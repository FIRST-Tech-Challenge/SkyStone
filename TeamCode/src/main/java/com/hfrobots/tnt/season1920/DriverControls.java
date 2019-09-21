/**
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.season1920;

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.LowPassFilteredRangeInput;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.ParametricScaledRangeInput;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.control.RangeInputButton;

import lombok.Builder;

@Builder
public class DriverControls {
    protected RangeInput driverLeftStickX;

    protected RangeInput driverLeftStickY;

    protected RangeInput driverRightStickX;

    protected RangeInput driverRightStickY;

    protected RangeInput driveForwardReverse;

    protected RangeInput driveStrafe;

    protected RangeInput driveRotate;

    protected DebouncedButton driverDpadUp;

    protected DebouncedButton driverDpadDown;

    protected DebouncedButton driverDpadLeft;

    protected DebouncedButton driverDpadRight;

    protected DebouncedButton driverXBlueButton;

    protected DebouncedButton driverBRedButton;

    protected DebouncedButton driverYYellowButton;

    protected DebouncedButton driverAGreenButton;

    protected DebouncedButton driverRightBumper;

    protected DebouncedButton driverLeftBumper;

    protected DebouncedButton lockButton;

    protected DebouncedButton unlockButton;

    protected OnOffButton driveInvertedButton;

    protected OnOffButton driveFastButton;

    protected OnOffButton driveBumpStrafeRightButton;

    protected OnOffButton driveBumpStrafeLeftButton;

    private NinjaGamePad driversGamepad;

    protected final float throttleGain = 0.7F;

    protected final float throttleExponent = 5; // MUST BE AN ODD NUMBER!

    protected final float throttleDeadband = 0;

    private final float lowPassFilterFactor = .92F;

    private OpenLoopMecanumKinematics kinematics;

    public void setGamepad(NinjaGamePad driversGamepad) {
        this.driversGamepad = driversGamepad;

        setupDriverControls();
    }

    public void setupCurvesAndFilters() {
        driveStrafe = new ParametricScaledRangeInput(
                new LowPassFilteredRangeInput(driverLeftStickX, lowPassFilterFactor),
                throttleDeadband, throttleGain, throttleExponent);

        driveForwardReverse = new ParametricScaledRangeInput(
                new LowPassFilteredRangeInput(driverLeftStickY, lowPassFilterFactor),
                throttleDeadband, throttleGain, throttleExponent);

        driveRotate = new LowPassFilteredRangeInput(driverRightStickX, lowPassFilterFactor);
    }

    private void setupDriverControls() {
        driverLeftStickX = driversGamepad.getLeftStickX();
        driverLeftStickY = driversGamepad.getLeftStickY();
        driverRightStickX = driversGamepad.getRightStickX();
        driverRightStickY = driversGamepad.getRightStickY();

        driverDpadDown = new DebouncedButton(driversGamepad.getDpadDown());
        driverDpadUp = new DebouncedButton(driversGamepad.getDpadUp());
        driverDpadLeft = new DebouncedButton(driversGamepad.getDpadLeft());
        driverDpadRight = new DebouncedButton(driversGamepad.getDpadRight());
        driverAGreenButton = new DebouncedButton(driversGamepad.getAButton());
        driverBRedButton = new DebouncedButton(driversGamepad.getBButton());
        driverXBlueButton = new DebouncedButton(driversGamepad.getXButton());
        driverYYellowButton = new DebouncedButton(driversGamepad.getYButton());
        driverLeftBumper = new DebouncedButton(driversGamepad.getLeftBumper());
        driverRightBumper = new DebouncedButton(driversGamepad.getRightBumper());
        lockButton = new DebouncedButton(driversGamepad.getLeftStickButton());
        unlockButton = new DebouncedButton(driversGamepad.getRightStickButton());
        driveFastButton = new RangeInputButton(driversGamepad.getLeftTrigger(), 0.65f);
        driveInvertedButton = new RangeInputButton(driversGamepad.getRightTrigger(), 0.65f);
        driveBumpStrafeLeftButton = driversGamepad.getLeftBumper();
        driveBumpStrafeRightButton = driversGamepad.getRightBumper();
    }

    public void periodicTask() {
        double x = - driveStrafe.getPosition(); // positive robot x axis is negative joystick axis
        double y = - driveForwardReverse.getPosition();
        double rot = - driveRotate.getPosition(); // positive robot z rotation (human-normal) is negative joystick x axis

        y = -y; // still need to figure this one out!

        // do this first, it will be cancelled out by bump-strafe
        if (!driveFastButton.isPressed()) {
            y /= 1.5;
            x /= 1.25;
            rot /= 1.5;
        }

        final boolean driveInverted;

        if (driveInvertedButton.isPressed()) {
            driveInverted = true;
        } else {
            driveInverted = false;
        }

        double xScaled = x;
        double yScaled = y;
        double rotateScaled = rot;

        // we check both bumpers - because both being pressed is driver 'panic', and we
        // don't want unexpected behavior!
        if (driveBumpStrafeLeftButton.isPressed() && !driveBumpStrafeRightButton.isPressed()) {
            xScaled = .6;
            yScaled = 0;
            rotateScaled = 0;
        } else if (driveBumpStrafeRightButton.isPressed() && !driveBumpStrafeLeftButton.isPressed()) {
            xScaled = -.6;
            yScaled = 0;
            rotateScaled = 0;
        }

        kinematics.driveCartesian(xScaled, yScaled, rotateScaled, driveInverted, 0.0);
    }

}
