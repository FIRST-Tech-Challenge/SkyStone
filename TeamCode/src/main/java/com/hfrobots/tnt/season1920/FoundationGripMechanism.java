/*
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
*/

package com.hfrobots.tnt.season1920;

import android.util.Log;

import com.google.common.base.Ticker;
import com.google.common.collect.Lists;
import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.hfrobots.tnt.corelib.drive.ServoUtil;
import com.hfrobots.tnt.corelib.drive.StallDetector;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.StopwatchDelayState;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;

import lombok.Setter;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class FoundationGripMechanism {

    Servo leftFoundationGripServo;
    Servo rightFoundationGripServo;

    public final static double RIGHT_GRIP_SERVO_UP = .823;  //THESE NEED THE VALUES WE MEASURED
    public final static double RIGHT_GRIP_SERVO_DOWN = .243;
    public final static double LEFT_GRIP_SERVO_UP = .136;
    public final static double LEFT_GRIP_SERVO_DOWN = .7;

    public FoundationGripMechanism(SimplerHardwareMap hardwareMap) {

        leftFoundationGripServo = hardwareMap.get(Servo.class, "leftGripServo");
        //ServoUtil.setupPwmForRevSmartServo(leftFoundationGripServo); // I am not sure we need, this, in the delivery mechanism class we use it for some servos but not others

        rightFoundationGripServo = hardwareMap.get(Servo.class, "rightGripServo");
        //ServoUtil.setupPwmForRevSmartServo(rightFoundationGripServo);  //not sure if needed

        // Move everything to initial physical state

        if (true) {
            up();
        }
    }

    public void up() {

        leftFoundationGripServo.setPosition(LEFT_GRIP_SERVO_UP);
        rightFoundationGripServo.setPosition(RIGHT_GRIP_SERVO_UP);
    }

    public void down() {
        leftFoundationGripServo.setPosition(LEFT_GRIP_SERVO_DOWN);
        rightFoundationGripServo.setPosition(RIGHT_GRIP_SERVO_DOWN);
    }

}
