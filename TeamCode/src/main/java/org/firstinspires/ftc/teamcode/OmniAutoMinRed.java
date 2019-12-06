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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto: AutoMinRed", group ="Auto")
public class OmniAutoMinRed extends OmniAutoClass
{
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

        robot.stackFromSide = HardwareOmnibot.RobotSide.LEFT;
        // Make sure the intake is out.
        robot.moveIntake(HardwareOmnibot.IntakePosition.EXTENDED);
        double endTime = timer.milliseconds() + 10000;
        while (!robot.intakeAtPosition(HardwareOmnibot.IntakePosition.EXTENDED) && (timer.milliseconds() < endTime) && (!isStopRequested())) {
            robot.resetReads();
        }

        // Set the zero for the extender for when we start teleop.  We should do this as late
        // as will get reliably called.
		sleep(1000);
        robot.finalAutoIntakePosition = HardwareOmnibot.IntakePosition.EXTENDED;
        robot.finalAutoIntakeZero = robot.getIntakeAbsoluteEncoder();
        robot.finalAutoLiftZero = robot.getLifterAbsoluteEncoder();
    }
}