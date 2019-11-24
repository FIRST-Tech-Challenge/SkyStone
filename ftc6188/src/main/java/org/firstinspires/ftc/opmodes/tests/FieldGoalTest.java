package org.firstinspires.ftc.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.robot.FieldGoalRobot;
import org.firstinspires.ftc.robotlib.state.Button;

@TeleOp(name="FieldGoal Test", group="Test")
public class FieldGoalTest extends OpMode
{
    private FieldGoalRobot robot;

    private Button toggleLimits;
    private Button setLimits;
    private Button servoMax;
    private Button servoMin;

    @Override
    public void init()
    {
        robot = new FieldGoalRobot(this.hardwareMap, this.telemetry, true);

        toggleLimits = new Button();
        setLimits = new Button();
        servoMax = new Button();
        servoMin = new Button();
    }

    @Override
    public void loop()
    {
        toggleLimits.input(gamepad1.x);
        setLimits.input(gamepad1.b);
        servoMax.input(gamepad1.dpad_up);
        servoMin.input(gamepad1.dpad_down);

        robot.armSystem.setVerticalPower(gamepad1.right_stick_y);
        robot.armSystem.setHorizontalPower(gamepad1.left_stick_y);

        if (setLimits.isPressed())
        {
            robot.armSystem.getVerticalLimitedMotor().setUpperLimit(robot.armSystem.getVerticalMotor().getCurrentPosition());
            robot.armSystem.getHorizontalLimitedMotor().setUpperLimit(robot.armSystem.getHorizontalMotor().getCurrentPosition());
        }

        if (toggleLimits.isPressed())
        {
            robot.armSystem.getVerticalLimitedMotor().setLimited(!robot.armSystem.getVerticalLimitedMotor().isLimited());
            robot.armSystem.getHorizontalLimitedMotor().setLimited(!robot.armSystem.getHorizontalLimitedMotor().isLimited());
        }

        if (servoMax.isPressed()) {robot.armGripSlide.setPosition(1);}
        else if (servoMin.isPressed()) {robot.armGripSlide.setPosition(0);}

        telemetry.update();
    }
}
