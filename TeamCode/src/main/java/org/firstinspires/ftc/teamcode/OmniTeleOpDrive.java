package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.toDegrees;

/**
 * Created by Ethan on 12/2/2016.
 */

//@TeleOp(name="Omni: TeleOpDrive", group ="TeleOp")
public class OmniTeleOpDrive extends OpMode {
    public OmniTeleOpDrive() {
        msStuckDetectInit = 10000;
    }

    public HardwareOmnibotDrive robot = new HardwareOmnibotDrive();

    @Override
    public void init() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        gamepad1.setJoystickDeadzone((float)robot.MIN_DRIVE_RATE);
        gamepad2.setJoystickDeadzone((float)robot.MIN_DRIVE_RATE);
        robot.init(hardwareMap);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
    }


    private double driverAngle = 0.0;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private double yPower;
    private double xPower;
    private double spin;
    private double gyroAngle;

    @Override
    public void start()
    {
    }

    @Override
    public void loop() {
        robot.resetReads();

        //left joystick is for moving
        //right joystick is for rotation
        yPower = -gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        spin = gamepad1.right_stick_x;
        gyroAngle = robot.readIMU();


        if (gamepad1.x) {
            // The driver presses X, then uses the left joystick to say what angle the robot
            // is aiming.  This will calculate the values as long as X is pressed, and will
            // not drive the robot using the left stick.  Once X is released, it will use the
            // final calculated angle and drive with the left stick.  Button should be released
            // before stick.  The default behavior of atan2 is 0 to -180 on Y Axis CCW, and 0 to
            // 180 CW.  This code normalizes that to 0 to 360 CCW from the Y Axis
            //robot.resetGyro();
            driverAngle = toDegrees(atan2(yPower, xPower)) - robot.readIMU();
            xPower = 0.0;
            yPower = 0.0;
            spin = 0.0;
        }

        robot.drive(speedMultiplier * xPower, speedMultiplier * yPower, spinMultiplier * spin, driverAngle, robot.defaultInputShaping);


        telemetry.addData("Y Power: ", yPower);
        telemetry.addData("X Power: ", xPower);
        telemetry.addData("Spin: ", spin);
        telemetry.addData("Offset Angle: ", driverAngle);
        telemetry.addData("Gyro Angle: ", gyroAngle);
        updateTelemetry(telemetry);
    }
}
