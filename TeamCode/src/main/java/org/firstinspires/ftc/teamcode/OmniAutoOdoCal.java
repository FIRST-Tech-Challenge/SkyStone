/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous(name="Auto: AutoOdoCal", group ="Auto")
public class OmniAutoOdoCal extends OmniAutoClass
{
    final double COUNTS_PER_CM = 241.245387;
    double horizontalTickOffset = 0;

    @Override
    public void runOpMode()
    {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        setupRobotParameters(4.0, 19.2);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();
        robot.resetReads();
        int leftWheelStart = robot.getLeftEncoderWheelPosition();
        int rightWheelStart = robot.getRightEncoderWheelPosition();
        int strafeWheelStart = robot.getStrafeEncoderWheelPosition();
        double startAngle = robot.readIMU();
        double endAngle = startAngle;
        double deltaAngle = 0.0;

        while(deltaAngle < 90 && opModeIsActive()) {
            if(deltaAngle < 60) {
                robot.drive(0, 0, -0.25, 0, false);
            } else {
                robot.drive(0, 0, -0.05, 0, false);
            }
            robot.resetReads();
            endAngle = robot.readIMU();
            deltaAngle = endAngle - startAngle;
        }
        robot.setAllDriveZero();
        sleep(1000);

        robot.resetReads();
        endAngle = robot.readIMU();
        deltaAngle = endAngle - startAngle;
        int leftDelta = Math.abs(robot.getLeftEncoderWheelPosition() - leftWheelStart);
        int rightDelta = Math.abs(robot.getRightEncoderWheelPosition() - rightWheelStart);
        double encoderDifference = leftDelta + rightDelta;

        double verticalEncoderTickOffsetPerDegree = encoderDifference/deltaAngle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_CM);

        int strafeDelta = robot.getStrafeEncoderWheelPosition() - strafeWheelStart;
        horizontalTickOffset = strafeDelta/Math.toRadians(deltaAngle);

        while(opModeIsActive()) {
            telemetry.addData("Wheel Base: ", wheelBaseSeparation);
            telemetry.addData("Horizontal Tick: ", horizontalTickOffset);
            telemetry.addData("Start Angle: ", startAngle);
            telemetry.addData("End Angle: ", endAngle);
            telemetry.addData("Delta Angle: ", deltaAngle);

            telemetry.addData("Start Left Wheel: ", leftWheelStart);
            telemetry.addData("Left Delta: ", leftDelta);
            telemetry.addData("Start Right Wheel: ", rightWheelStart);
            telemetry.addData("Right Delta: ", rightDelta);
            telemetry.addData("Start Strafe Wheel: ", strafeWheelStart);
            telemetry.addData("Strafe Delta: ", strafeDelta);

            telemetry.update();
        }
    }
}