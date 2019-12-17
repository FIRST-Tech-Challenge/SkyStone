package org.firstinspires.ftc.opmodes.competition.auto.TimeAuto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.AutoDirection;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;

@Autonomous(name="Time Based Auto", group="CompAuto")
public class SiBorgsMecanumAutoTime extends LinearOpMode
{
    // Robot
    private SiBorgsMecanumRobot robot;
    private ElapsedTime elapsedTime;

    // Fields
    private static final double VELOCITY = 0.5;

    // Buttons
    private Button capstoneOpen;
    private Button capstoneClose;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new SiBorgsMecanumRobot(this.hardwareMap, this.telemetry);
        robot.changeBackgroundColor(Color.BLUE);

        elapsedTime = new ElapsedTime();
        capstoneOpen = new Button();
        capstoneClose = new Button();

        /** Before the auto period starts the drivers should load a capstone into the arm **/
        while (!isStarted())
        {
            capstoneOpen.input(gamepad1.dpad_up || gamepad2.dpad_down || gamepad1.y || gamepad2.y);
            capstoneClose.input(gamepad1.dpad_down || gamepad2.dpad_down || gamepad1.a || gamepad2.a);

            if (capstoneOpen.onPress()) { robot.armGripSlide.setPosition(ServoState.UP); }
            else if (capstoneClose.onPress()) { robot.armGripSlide.setPosition(ServoState.DOWN); }

            robot.armCrane.setVerticalPower(-gamepad1.left_stick_y);
            robot.armCrane.setHorizontalPower(gamepad1.right_stick_y);

            telemetry.addData("ADD CAPSTONE TO SERVO (G1-DPAD) + (G1-LStick)", robot.armGripSlide.getState()); telemetry.update();
        }
        telemetry.addData("START OF AUTO PERIOD", ""); telemetry.update();
        /** Auto period now starts **/

        telemetry.addData("1 Sec", timePos(AutoDirection.FRONT, 1));
        sleep(5000);
        telemetry.addData("2 Sec", timePos(AutoDirection.FRONT, 2));
        sleep(5000);
        telemetry.addData("3 Sec", timePos(AutoDirection.FRONT, 3));
        sleep(5000);
        telemetry.addData("4 Sec", timePos(AutoDirection.FRONT, 4));
        sleep(5000);
        telemetry.addData("5 Sec", timePos(AutoDirection.FRONT, 5));
    }

    private double timePos(AutoDirection course, double time)
    {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        double startTime = elapsedTime.seconds();
        robot.drivetrain.autoPositionByTime(course, time, VELOCITY);
        return elapsedTime.seconds() - startTime;
    }

    private double convertDistanceInToSeconds(double distanceIn)
    {
        return (0.0366599496 * distanceIn) + 0.0851922729;
    }
}
