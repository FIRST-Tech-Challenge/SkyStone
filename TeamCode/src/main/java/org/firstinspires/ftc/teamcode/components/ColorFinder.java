/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
public class ColorFinder extends BotComponent {

    private String leftSensorName = "";
    private String rightSensorName = "";

    public ColorSensor leftColor;
    public DistanceSensor leftDistance;
    public ColorSensor rightColor;
    public DistanceSensor rightDistance;
    public boolean stoneColor;

    public ColorFinder() {
    }

    public ColorFinder(Logger aLogger, OpMode aOpMode, String aLeftSensorName, String aRightSensorName) {
        super(aLogger, aOpMode);

        leftSensorName = aLeftSensorName;
        rightSensorName = aRightSensorName;

    }

    public void init() {

        leftColor = opMode.hardwareMap.get(ColorSensor.class, leftSensorName);
        leftDistance = opMode.hardwareMap.get(DistanceSensor.class, leftSensorName);
        rightColor = opMode.hardwareMap.get(ColorSensor.class, rightSensorName);
        rightDistance = opMode.hardwareMap.get(DistanceSensor.class, rightSensorName);

        if (rightColor.red() == )

        if (leftColor != null  &&  rightColor != null) {
            isAvailable = true;
        }

        logger.logInfo("ColorFinder", "isAvailable: %b", isAvailable);
    }

}

