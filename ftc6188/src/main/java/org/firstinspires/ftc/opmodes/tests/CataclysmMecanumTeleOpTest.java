package org.firstinspires.ftc.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.robot.CataclysmMecanumRobot;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ServoState;

@Disabled
@TeleOp(name="Cataclysm TeleOp Test", group="Test")
public class CataclysmMecanumTeleOpTest extends OpMode
{
    private CataclysmMecanumRobot robot;

    private Button blockGrabberOpen;
    private Button blockGrabberClose;
    private Button deliveryOpen;
    private Button deliveryClose;

    @Override
    public void init()
    {
        robot = new CataclysmMecanumRobot(this.hardwareMap, this.telemetry);

        blockGrabberOpen = new Button();
        blockGrabberClose = new Button();
        deliveryOpen = new Button();
        deliveryClose = new Button();
    }

    @Override
    public void loop()
    {
        blockGrabberOpen.input(gamepad1.dpad_up);
        blockGrabberClose.input(gamepad1.dpad_down);
        deliveryOpen.input(gamepad1.y);
        deliveryClose.input(gamepad1.a);

        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        robot.drivetrain.setCourse(course);
        robot.drivetrain.setVelocity(velocity);
        robot.drivetrain.setRotation(-gamepad1.left_stick_x);

        if (blockGrabberOpen.onPress()) { robot.servoBlockGrabber.setPosition(ServoState.UP); }
        else if (blockGrabberClose.onPress()) { robot.servoBlockGrabber.setPosition(ServoState.DOWN); }

        if (deliveryOpen.onPress())
        {
            robot.servoDeliveryLeft.setPosition(ServoState.UP);
            robot.servoDeliveryRight.setPosition(ServoState.UP);
        }
        else if (deliveryClose.onPress())
        {
            robot.servoDeliveryLeft.setPosition(ServoState.DOWN);
            robot.servoDeliveryRight.setPosition(ServoState.DOWN);
        }

        robot.armIntakeRight.setPower(gamepad1.right_trigger);
        robot.armIntakeLeft.setPower(gamepad1.left_trigger);

        robot.informationTelemetry();
    }
}
