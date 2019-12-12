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

import android.util.Log;

import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.control.ChaosNinjaLandingState;
import com.hfrobots.tnt.corelib.control.KonamiCode;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.drive.mecanum.DriveConstants;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveREV;
import com.hfrobots.tnt.corelib.metrics.StatsDMetricSampler;
import com.hfrobots.tnt.corelib.util.RealSimplerHardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@TeleOp(name="00 Skystone Teleop")
@SuppressWarnings("unused")
public class SkystoneTeleop extends OpMode {
    private OpenLoopMecanumKinematics kinematics;

    private RoadRunnerMecanumDriveREV driveBase;

    private DriverControls driverControls;

    private OperatorControls operatorControls;

    private DeliveryMechanism deliveryMechanism;

    private StatsDMetricSampler metricSampler;

    private Ticker ticker;

    private KonamiCode konamiCode;

    private ChaosNinjaLandingState chaosNinja;
    private NinjaGamePad driversGamepad;
    private NinjaGamePad operatorsGamepad;

    @Override
    public void init() {
        ticker = createAndroidTicker();

        RealSimplerHardwareMap simplerHardwareMap = new RealSimplerHardwareMap(this.hardwareMap);
        driveBase = new RoadRunnerMecanumDriveREV(new SkystoneDriveConstants(), simplerHardwareMap, false);
        kinematics = new OpenLoopMecanumKinematics(driveBase);

        driversGamepad = new NinjaGamePad(gamepad1);

        driverControls = DriverControls.builder().driversGamepad(driversGamepad)
                .kinematics(kinematics)
                .build();
        deliveryMechanism = new DeliveryMechanism(simplerHardwareMap, telemetry, ticker);

        operatorsGamepad = new NinjaGamePad(gamepad2);

        operatorControls = OperatorControls.builder().operatorsGamepad(operatorsGamepad)
                .deliveryMechanism(deliveryMechanism)
                .build();

        //chaosNinja = new ChaosNinjaLandingState(driversGamepad, telemetry);
        //konamiCode = new KonamiCode(driversGamepad, null, ticker, telemetry);
    }

//    @Override
//    public void init_loop() {
//        super.init_loop();
//
//        konamiCode.periodicTask();
//    }

    private Ticker createAndroidTicker() {
        return new Ticker() {
            public long read() {
                return android.os.SystemClock.elapsedRealtimeNanos();
            }
        };
    }

    @Override
    public void start() {
        super.start();

        if (chaosNinja != null) {
            if (chaosNinja.isMetricsActivated()) {
                Log.i(LOG_TAG, "Metrics requested, enabling");
                metricSampler = new StatsDMetricSampler(hardwareMap, driversGamepad, operatorsGamepad);
            } else {
                Log.i(LOG_TAG, "No metrics requested, not enabling");
                metricSampler = null;
            }
        }
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void loop() {
        driverControls.periodicTask();
        operatorControls.periodicTask();

        if (metricSampler != null) {
            metricSampler.doSamples();
        }
    }
}
