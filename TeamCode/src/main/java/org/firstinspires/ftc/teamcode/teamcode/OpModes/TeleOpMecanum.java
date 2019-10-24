package org.firstinspires.ftc.teamcode.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Outtake;

@TeleOp(name="TeleOpFinal", group= "Arcade")
public class TeleOpMecanum extends OpMode {

    DriveTrain drive = new DriveTrain();
    Intake intake = new Intake();
    Outtake outtake = new Outtake();

    double leftStickY;
    double leftStickX;
    double direction;
    double velocity;
    double speed;
    double speedProp = 1.0;
    boolean pastX = false;

    @Override
    public void init() {

        drive.fl = hardwareMap.dcMotor.get("fl");
        drive.fr = hardwareMap.dcMotor.get("fr");
        drive.bl = hardwareMap.dcMotor.get("bl");
        drive.br = hardwareMap.dcMotor.get("br");

        drive.fl.setDirection(DcMotor.Direction.FORWARD);
        drive.fr.setDirection(DcMotor.Direction.REVERSE);
        drive.bl.setDirection(DcMotor.Direction.FORWARD);
        drive.br.setDirection(DcMotor.Direction.REVERSE);

        drive.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.initIntake(this);
        outtake.initOuttake(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.runtime.reset();

    }

    //Main Loop
    @Override
    public void loop() {

        speed = gamepad1.right_stick_x;

        telemetry.addData("Status", "Run Time: " + drive.runtime.toString());
        telemetry.addData("Motor Position", "Motor Rotation", +speed);

        if (Math.abs(gamepad1.left_stick_y) > .05) {
            leftStickY = gamepad1.left_stick_y;
        } else {
            leftStickY = 0;
        }

        if (Math.abs(gamepad1.left_stick_x) > .05) {
            leftStickX = gamepad1.left_stick_x;
        } else {
            leftStickX = 0;
        }

        if (gamepad1.x != pastX) {
            pastX = gamepad1.x;
            if (gamepad1.x) {
                if (speedProp == 1) {
                    speedProp = 0.5;
                } else {
                    speedProp = 1;
                }
            }
        }

        telemetry.addData("Velocity : ", velocity);

        telemetry.addData("Direction : ", direction);

        telemetry.addData("Speed : ", speed);

        velocity = gamepad1.left_stick_y;

        direction = gamepad1.left_stick_x;

        speed = gamepad1.right_stick_x;

        if (Math.abs(gamepad1.right_stick_x) < 0.075 ) {
            speed = 0;
        }

        if (Math.abs(gamepad1.left_stick_x) > 0.075 ||
                Math.abs(gamepad1.left_stick_y) >
                        0.075 || Math.abs(gamepad1.right_stick_x)
                > 0.075) {
            drive.fl.setPower((velocity - direction) - speed);
            drive.fr.setPower((velocity + direction) + speed);
            drive.bl.setPower((velocity + direction) - speed);
            drive.br.setPower((velocity - direction) + speed);
        }

        else {
            drive.snowWhite();
        }

        intake.compliantIntake_TeleOp(this);
        outtake.outTake_TeleOp(this);
        telemetry.addData("Halfing Speed : ", pastX);
        telemetry.addData("Encoded Acceleration : ", drive.getEncodedAccel());

        telemetry.addData("Get Holon : ",
                "FR :" + drive.getHolon(drive.fr) +
                        "BL : " + drive.getHolon(drive.bl) +
                        "BR : " + drive.getHolon(drive.br));
        telemetry.update();



    }
}
