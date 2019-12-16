package org.firstinspires.ftc.opmodes.competition.auto.FullAuto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.AutoDirection;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;

@Disabled
@Autonomous(name="Auto Full V-AutoComp", group="AutoComp")
public class SiBorgsMecanumAutoFull extends LinearOpMode
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

        /** Commands **/
        // Plan: forward, grab platform, reverse, left to front of platform, front, right to push, left under bridge, forward
        robot.drivetrain.autoPositionByEncoder(AutoDirection.FRONT, 29.5, VELOCITY);

        robot.platformServo.setPosition(ServoState.DOWN);
        robot.drivetrain.autoPositionByEncoder(AutoDirection.REAR, 29.5, VELOCITY);
        robot.platformServo.setPosition(ServoState.UP);

        robot.drivetrain.autoPositionByEncoder(AutoDirection.LEFT, 24, VELOCITY);
        robot.drivetrain.autoPositionByEncoder(AutoDirection.FRONT, 22, VELOCITY);
        robot.drivetrain.autoPositionByEncoder(AutoDirection.RIGHT, 10, VELOCITY);
        robot.drivetrain.autoPositionByEncoder(AutoDirection.LEFT, 36, VELOCITY);
        robot.drivetrain.autoPositionByEncoder(AutoDirection.FRONT, 9, VELOCITY);

        // Pause then end the op mode safely
        sleep(1000);
        requestOpModeStop();
    }
}
