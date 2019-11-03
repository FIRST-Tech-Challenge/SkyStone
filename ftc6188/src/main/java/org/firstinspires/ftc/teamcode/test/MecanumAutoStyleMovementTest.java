package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ToggleInt;

@TeleOp(name="Movement Test", group="Auto")
public class MecanumAutoStyleMovementTest extends OpMode
{
    private MecanumRobot robot;

    private Button leftMove;
    private Button rightMove;
    private Button forwardMove;
    private Button reverseMove;
    private Button clockwiseRotate;
    private Button counterClockRotate;
    private ToggleInt targetDistance;

    private double targetDistanceDouble = 0.0;

    @Override
    public void init()
    {
        robot = new MecanumRobot(this.hardwareMap, this.telemetry, false);
        leftMove = new Button();
        rightMove = new Button();
        forwardMove = new Button();
        reverseMove = new Button();
        clockwiseRotate = new Button();
        counterClockRotate = new Button();
        targetDistance = new ToggleInt(12, 1);
    }

    @Override
    public void loop()
    {
        leftMove.input(gamepad1.dpad_left);
        rightMove.input(gamepad1.dpad_right);
        forwardMove.input(gamepad1.dpad_up);
        reverseMove.input(gamepad1.dpad_down);
        clockwiseRotate.input(gamepad1.right_bumper);
        counterClockRotate.input(gamepad1.left_bumper);
        targetDistance.input(gamepad1.x);

        targetDistanceDouble = targetDistance.output() * 1.0;

        if (leftMove.onPress())
        {
            robot.robotMove(270, 1, 0, targetDistanceDouble);
        }

        if (rightMove.onPress())
        {
            robot.robotMove(90, 1, 0, targetDistanceDouble);
        }

        if (forwardMove.onPress())
        {
            robot.robotMove(0, 1, 0, targetDistanceDouble);
        }

        if (reverseMove.onPress())
        {
            robot.robotMove(180, 1, 0, targetDistanceDouble);
        }

        if (clockwiseRotate.onPress())
        {
            robot.robotMove(0, 1, 90, 0);
        }

        if (counterClockRotate.onPress())
        {
            robot.robotMove(0, 1, -90, 0);
        }

        robot.informationUpdate();
    }
}
