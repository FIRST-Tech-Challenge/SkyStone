package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.hardwaremap.MecanumHardwareMap;

@TeleOp(name="Basic Mecanum (12069)", group="Linear Opmode")
public class MecanumTeleOp extends OpMode
{
    private MecanumHardwareMap robotHardware;
    private ElapsedTime elapsedTime;
    private boolean rightMotion = false;

    @Override
    public void init()
    {
        robotHardware = new MecanumHardwareMap(this.hardwareMap);
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void init_loop()
    {
        telemetry.addData("Status", "Init Loop");
        telemetry.update();
    }

    @Override
    public void start()
    {
        elapsedTime.reset();
    }

    @Override
    public void loop()
    {
        double course;
        double velocity;
        if (rightMotion) {
            course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
            velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        } else {
            course = Math.atan2(-gamepad1.right_stick_x, gamepad1.right_stick_y) - Math.PI/2;
            velocity = Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x);
        }

        robotHardware.drivetrain.setCourse(course);
        robotHardware.drivetrain.setVelocity(velocity);
        robotHardware.drivetrain.setRotation(-gamepad1.left_stick_x);

        /**
        if (gamepad1.dpad_down)
        {
            robotHardware.servoBuildClawLeft.setPosition(120);
            robotHardware.servoBuildClawRight.setPosition(120);
        }

        if (gamepad1.dpad_up)
        {
            robotHardware.servoBuildClawLeft.setPosition(0);
            robotHardware.servoBuildClawRight.setPosition(0);
        }
         **/

        telemetry.addData("Status", "Loop: " + elapsedTime.toString());
        telemetry.addData("Course", course);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Rotation", -gamepad1.left_stick_x);
        telemetry.update();
    }

    @Override
    public void stop()
    {
        telemetry.addData("Status", "Stop");
        telemetry.update();
    }
}
