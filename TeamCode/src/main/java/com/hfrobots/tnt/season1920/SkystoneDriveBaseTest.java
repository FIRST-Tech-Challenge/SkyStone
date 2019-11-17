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

import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.drive.mecanum.DriveConstants;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveREV;
import com.hfrobots.tnt.corelib.metrics.StatsDMetricSampler;
import com.hfrobots.tnt.corelib.util.RealSimplerHardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Skystone Drive Test",  group="Utilities")
@SuppressWarnings("unused")
public class SkystoneDriveBaseTest extends OpMode {
    private OpenLoopMecanumKinematics kinematics;

    private RoadRunnerMecanumDriveREV driveBase;


    private StatsDMetricSampler metricSampler;
    private NinjaGamePad driversGamepad;

    @Override
    public void init() {
        RealSimplerHardwareMap simplerHardwareMap = new RealSimplerHardwareMap(this.hardwareMap);
        driveBase = new RoadRunnerMecanumDriveREV(new SkystoneDriveConstants(), simplerHardwareMap, false);
        kinematics = new OpenLoopMecanumKinematics(driveBase);

        driversGamepad = new NinjaGamePad(gamepad1);


        NinjaGamePad operatorsGamepad = new NinjaGamePad(gamepad2);

        metricSampler = new StatsDMetricSampler(hardwareMap, driversGamepad, operatorsGamepad);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void loop() {
        kinematics.driveCartesian(driversGamepad.getLeftStickX().getPosition(),0, 0, false, 0,
                driversGamepad.getAButton().isPressed());

        if (metricSampler != null) {
            metricSampler.doSamples();
        }
    }
}
