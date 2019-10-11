package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.*;

/**
 * Created by Ethan on 12/2/2016.
 */

@TeleOp(name="Omni: TeleOp", group ="TeleOp")
public class OmniTeleOp extends OpMode {

    public HardwareOmnibot robot = new HardwareOmnibot();

    @Override
    public void init() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        robot.init(hardwareMap);
        robot.setInputShaping(true);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
    }

    private double driverAngle = 0.0;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private boolean aHeld = false;
    private boolean bHeld = false;
    private boolean yHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean aPressed;
    private boolean bPressed;
    private boolean yPressed;
    private boolean upPressed;
    private boolean downPressed;
    private double yPower;
    private double xPower;
    private double spin;
    private double gyroAngle;
    private double liftPower;
    private double extendPower;
    private double collectPower;
	private LiftPosition liftTargetHeight = LiftPosition.STOWED;

    @Override
    public void start()
    {
    }

    @Override
    public void loop() {
        //left joystick is for moving
        //right joystick is for rotation
        gyroAngle = robot.readIMU();

        yPower = -gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        spin = gamepad1.right_stick_x;
        aPressed = gamepad1.a;
        bPressed = gamepad1.b;
        yPressed = gamepad1.y;
        upPressed = gamepad1.dpad_up;
        downPressed = gamepad1.dpad_down;

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

        if(!aHeld && aPressed)
        {
            aHeld = true;
            robot.toggleFingers();
        } else if(!aPressed) {
            aHeld = false;
        }

        if(!bHeld && bPressed)
        {
            bHeld = true;
            robot.toggleClaw();
        } else if(!bPressed) {
            bHeld = false;
        }

        if(!yHeld && yPressed)
        {
            yHeld = true;
            robot.setLiftHeight(liftTargetHeight);
        } else if(!yPressed) {
            yHeld = false;
        }

        if(!upHeld && upPressed)
        {
            upHeld = true;
            liftTargetHeight = LiftPosition.addStone(liftTargetHeight);
        } else if (!upPressed) {
			upHeld = false;
		}

        if(!downHeld && downPressed)
        {
            downHeld = true;
            liftTargetHeight = LiftPosition.removeStone(liftTargetHeight);
        } else if (!upPressed) {
			downHeld = false;
		}

        robot.drive(speedMultiplier * xPower, speedMultiplier * yPower, spinMultiplier * spin, driverAngle);

		telemetry.addData("Lift Target Height: ", liftTargetHeight.toString());
        telemetry.addData("Y Power: ", yPower);
        telemetry.addData("X Power: ", xPower);
        telemetry.addData("Spin: ", spin);
        telemetry.addData("Offset Angle: ", driverAngle);
        telemetry.addData("Gyro Angle: ", gyroAngle);
        telemetry.addData("Front Left Encoder: ", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Encoder: ", robot.frontRight.getCurrentPosition());
        telemetry.addData("Rear Left Encoder: ", robot.rearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Encoder: ", robot.rearRight.getCurrentPosition());
        telemetry.addData("Lifter Encoder: ", robot.lifter.getCurrentPosition());
        telemetry.addData("Extender Encoder: ", robot.extender.getCurrentPosition());
        updateTelemetry(telemetry);
    }

    @Override
    public void stop() {
        robot.stopGroundEffects();
    }
}
