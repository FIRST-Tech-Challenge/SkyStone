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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDDriveStraightTask implements RobotControl {
    @Override
    public void prepare() {
        pidDrive = new PIDController(0.1, -0.001, 0);
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 1);
        pidDrive.setInputRange(-5, 5);
        pidDrive.enable();
    }

    @Override
    public void execute() {
        double error = navigator.getWorldX();
        double correction = pidDrive.performPID(error);
        robot.setMotorPower(0.5 + correction, 0.5 - correction, 0.5 + correction, 0.5 - correction);
    }

    @Override
    public void cleanUp() {
        robot.setMotorPower(0, 0, 0, 0);
    }

    @Override
    public boolean isDone() {
        return (distance < navigator.getWorldY());
    }

    private RobotHardware robot;
    private RobotNavigator navigator;
    private PIDController pidDrive;
    private double distance;


    public PIDDriveStraightTask(RobotHardware robot, RobotNavigator navigator, double distance) {
        this.robot = robot;
        this.navigator = navigator;
        this.distance = distance;

    }
}