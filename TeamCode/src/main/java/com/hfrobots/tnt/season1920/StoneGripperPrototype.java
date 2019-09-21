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
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.util.NamedDeviceMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * An OpMode that allows you to test any/all of the servos on a robot
 *
 * gamepad1.left_stick.y = proportional adjustment
 * gamepad1.dpad up/down = micro adjustment
 * gamepad1.right_bumper = cycle through all configured servos
 *
 * current servo name and position are displayed in driver station telemetry
 *
 */
@TeleOp(name="Stone Gripper Prototype", group="Utilities")
@SuppressWarnings("unused")
public class StoneGripperPrototype extends OpMode {
    private double leftOpenPos = 0.655;

    private double leftGripPos = .90;

    private double rotateFwdPos = 0.29;

    private double rotateBackPos = 0.73;

    private DebouncedButton gripButton;

    private DebouncedButton releaseButton;

    private DebouncedButton rotateFwdButton;

    private DebouncedButton rotateBackButton;

    private boolean gripping = false;

    private Servo leftServo;

    private Servo rotateServo;

    @Override
    public void init() {
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rotateServo = hardwareMap.get(Servo.class, "rightServo");

        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);
        gripButton = new DebouncedButton(ninjaGamePad.getDpadUp());
        releaseButton = new DebouncedButton(ninjaGamePad.getDpadDown());

        rotateFwdButton = new DebouncedButton(ninjaGamePad.getDpadRight());
        rotateBackButton = new DebouncedButton(ninjaGamePad.getDpadLeft());
    }

    @Override
    public void loop() {
        if (gripButton.getRise()) {
            gripping = true;
        } else if (releaseButton.getRise()) {
            gripping = false;
        }

        if (gripping) {
            leftServo.setPosition(leftGripPos);
        } else {
            leftServo.setPosition(leftOpenPos);
        }

        if (rotateBackButton.getRise()) {
            rotateServo.setPosition(rotateBackPos);
        } else if (rotateFwdButton.getRise()) {
            rotateServo.setPosition(rotateFwdPos);
        }
    }
}
