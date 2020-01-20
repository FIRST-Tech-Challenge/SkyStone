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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



public class DistanceSensorBot extends PinchArmBot {

    protected DistanceSensor frontSensor = null;
    protected DistanceSensor backSensor = null;

    public DistanceSensorBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        // initialize the sensor for skystone quarry detection
        frontSensor = hwMap.get(DistanceSensor.class, "distance_front");
        backSensor = hwMap.get(DistanceSensor.class, "distance_back");
    }

    public double getDistanceFront() {
        opMode.telemetry.addData("range", String.format("%.01f cm", frontSensor.getDistance(DistanceUnit.CM)));

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) frontSensor;
        // Rev2mDistanceSensor specific methods.
        opMode.telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        opMode.telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

        opMode.telemetry.update();
        return frontSensor.getDistance(DistanceUnit.CM);
    }

    public double getDistanceBack() {
        opMode.telemetry.addData("range", String.format("%.01f cm", backSensor.getDistance(DistanceUnit.CM)));

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) backSensor;
        // Rev2mDistanceSensor specific methods.
        opMode.telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        opMode.telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

        opMode.telemetry.update();
        return backSensor.getDistance(DistanceUnit.CM);
    }


    public void driveUntilDistance(double distance, double power, int sensor) {

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (sensor == 0) {
            if (getDistanceFront() > distance) {

                do {
                    leftFront.setPower(- power);
                    rightFront.setPower(power);
                    leftRear.setPower(power);
                    rightRear.setPower(- power);
                }
                while (getDistanceFront() > distance);
            } else {

                do {
                    leftFront.setPower(power);
                    rightFront.setPower(- power);
                    leftRear.setPower(- power);
                    rightRear.setPower(power);
                }
                while (getDistanceFront() < distance);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        } else if (sensor == 1) {
            if (getDistanceBack() > distance) {

                do {
                    leftFront.setPower(- power);
                    rightFront.setPower(power);
                    leftRear.setPower(power);
                    rightRear.setPower(- power);
                }
                while (getDistanceBack() > distance);
            } else {

                do {
                    leftFront.setPower(power);
                    rightFront.setPower(- power);
                    leftRear.setPower(- power);
                    rightRear.setPower(power);
                }
                while (getDistanceFront() < distance);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        }


    }

    public void ghettoGyro() {

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (getDistanceFront() < getDistanceBack()) {
            do {
                leftFront.setPower(0.1);
                rightFront.setPower(-0.1);
                leftRear.setPower(0.1);
                rightRear.setPower(-0.1);
            }
            while (getDistanceFront() < getDistanceBack());
        } else if (getDistanceFront() > getDistanceBack()) {
            do {
                leftFront.setPower(-0.1);
                rightFront.setPower(0.1);
                leftRear.setPower(-0.1);
                rightRear.setPower(0.1);
            }
            while (getDistanceFront() > getDistanceBack());
        }

    }
}
