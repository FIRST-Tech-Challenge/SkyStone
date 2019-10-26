/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.demo;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * copied from SensorREV2mDistance in the samples directory. modified to use four lidar sensors instead of one.
 */
@TeleOp(name = "Sensor: REV2mDistance", group = "Sensor")
public class SensorREV2mDistance extends LinearOpMode {

    private DistanceSensor rangeFront;
    private DistanceSensor rangeBack;
    private DistanceSensor rangeLeft;
    private DistanceSensor rangeRight;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        rangeFront = hardwareMap.get(DistanceSensor.class, "sensor_front");
        rangeBack = hardwareMap.get(DistanceSensor.class, "sensor_back");
        rangeLeft = hardwareMap.get(DistanceSensor.class, "sensor_left");
        rangeRight = hardwareMap.get(DistanceSensor.class, "sensor_right");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)rangeFront;
        Rev2mDistanceSensor sensorTimeOfFlight0 = (Rev2mDistanceSensor)rangeBack;
        telemetry.addData("Front, Back", String.format("init 1:%s, 2:%s", rangeFront.getDeviceName(), rangeBack.getDeviceName()));

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("deviceName", String.format("run 1:%s, 2:%s", rangeFront.getDeviceName(), rangeBack.getDeviceName()));
            telemetry.addData("rangeFront", String.format("%.01f cm", rangeFront.getDistance(DistanceUnit.CM)));
            telemetry.addData("rangeBack", String.format("%.01f cm", rangeBack.getDistance(DistanceUnit.CM)));
            telemetry.addData("rangeLeft", String.format("%.01f cm", rangeLeft.getDistance(DistanceUnit.CM)));
            telemetry.addData("rangeRight", String.format("%.01f cm", rangeRight.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}