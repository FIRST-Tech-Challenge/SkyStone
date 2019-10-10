package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.hardwaremap.HeadingableMecanumHardwareMap;

@TeleOp (name="Headingable Mecanum TeleOp", group="Headingable")
public class HeadingableMecanumTeleOp extends OpMode
{
    private static final double HEADING_COEFF = 0.005;
    private HeadingableMecanumHardwareMap robotHardware;

    private double desiredHeading = 0;
    private ElapsedTime rotationTimer;
    private ElapsedTime elapsedTime;

    @Override
    public void init()
    {
        robotHardware = new HeadingableMecanumHardwareMap(this.hardwareMap);
        rotationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void start()
    {
        elapsedTime.reset();
    }

    @Override
    public void loop()
    {
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);

        desiredHeading += -gamepad1.left_stick_x*rotationTimer.time()*HEADING_COEFF;
        rotationTimer.reset();

        if (gamepad1.left_bumper)
        {
            desiredHeading += -Math.PI/4;
        }
        if (gamepad1.right_bumper)
        {
            desiredHeading += Math.PI/4;
        }
        if (gamepad1.y)
        {
            robotHardware.drivetrain.setExtrinsicOffset(desiredHeading);
        }

        //toggles the robots headingless ability maybe
        if (gamepad1.left_stick_button)
        {
            robotHardware.drivetrain.setExtrinsic(!robotHardware.drivetrain.getExtrinsic());
        }

        robotHardware.drivetrain.setCourse(course);
        robotHardware.drivetrain.setVelocity(velocity);
        robotHardware.drivetrain.setTargetHeading(desiredHeading);
        robotHardware.drivetrain.updateHeading();

        telemetry.addData("Status", "Loop: " + rotationTimer.toString());
        telemetry.addData("Course", course);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Rotation", -gamepad1.left_stick_x);
        telemetry.addData("Desired Heading", desiredHeading);
        telemetry.addData("Extrinsic Offset", robotHardware.drivetrain.getExtrinsicOffset());
        telemetry.update();
    }
}
