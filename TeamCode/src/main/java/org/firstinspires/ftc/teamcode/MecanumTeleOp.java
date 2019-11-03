package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.GyroSensor;

@TeleOp(name="Mecanum TeleOp", group="Linear Opmode")
//@Disabled
public class MecanumTeleOp extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    // defining back right wheel
    private DcMotor left1;
    // defining back left wheel
    private DcMotor left2;
    // defining front right wheel
    private DcMotor right1;
    // defining back left wheel
    private DcMotor right2;

    private ColorSensor color;
    // only for encoder use

    private DcMotor intakeLeft;
    // only for motor intake left motor
    private DcMotor intakeRight;
    // only for motor outtake right motor
    private Servo outtakeLeft;
    // only for motor outtake left
    private Servo outtakeRight;
    // only for motor outtake right

    private DcMotor liftRight;
    // only for elevator right motor
    private DcMotor liftLeft;
    // only for elevator left motor

    private Servo grabRight;
    // only for grabber right
    private Servo grabLeft;
    // only for grabber left

    private Servo nubGrabRight;
    // only for the nub grabber right
    private Servo nubGrabLeft;
    // only for the nub grabber left


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("leftFront/odometerLeftY");
        left2 = hardwareMap.dcMotor.get("leftBack");
        right1 = hardwareMap.dcMotor.get("rightFront/odometerRightY");
        right2 = hardwareMap.dcMotor.get("rightBack/odometerX");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        liftRight = hardwareMap.dcMotor.get("liftRight");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");

        grabLeft = hardwareMap.servo.get("grabLeft");
        grabRight = hardwareMap.servo.get("grabRight");

        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("right", grabRight.getPosition());
        telemetry.addData("left", grabRight.getPosition());
        telemetry.update();

        grabRight.setPosition(0.220);
        grabLeft.setPosition(0.665);

        boolean prevPos = false;
        boolean pos = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("leftOdo", left1.getCurrentPosition());
            telemetry.addData("rightOdo", right1.getCurrentPosition());
            telemetry.addData("alignOdo", right2.getCurrentPosition());
            telemetry.addData("rightGrab", grabRight.getPosition());
            telemetry.addData("leftGrab", grabLeft.getPosition());
            telemetry.update();

            float vertical = -(gamepad1.left_stick_y);
            float horizontal = gamepad1.left_stick_x;
            float pivot = gamepad1.right_stick_x;

            setRight1Power(-pivot + (vertical - horizontal));
            setRight2Power(-pivot + (vertical + horizontal));
            setLeft1Power(pivot + (vertical + horizontal));
            setLeft2Power(pivot + (vertical - horizontal));

            left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (gamepad2.right_bumper) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(1);
                telemetry.addData("intakeLeft", intakeLeft);
                telemetry.addData("intakeRight", intakeRight);
            } else {
                intakeRight.setPower(0);
                intakeLeft.setPower(0);
            }

            if (gamepad2.left_bumper) {
                intakeLeft.setPower(1);
                intakeRight.setPower(-1);
                telemetry.addData("intakeLeft", intakeLeft);
                telemetry.addData("intakeRight", intakeRight);
            } else {
                intakeRight.setPower(0);
                intakeLeft.setPower(0);
            }
            telemetry.addData("right", grabRight.getPosition());
            telemetry.addData("left", grabRight.getPosition());
            telemetry.update();

            if (!prevPos && gamepad2.x) {
                if (!pos) {
                    grabRight.setPosition(.753);
                    grabLeft.setPosition(.159);
                    pos = !pos;
                } else {
                    grabRight.setPosition(0.220);
                    grabLeft.setPosition(0.665);
                    pos = !pos;
                }
                prevPos = true;
            } else if (!gamepad2.x) {
                prevPos = false;
            }
        }
    }

    void setLeft2Power(double n){
        left2.setPower(n);
    }
    void setLeft1Power(double n){
        left1.setPower(n);
    }
    void setRight1Power(double n){
        right1.setPower(n);
    }
    void setRight2Power(double n){
        right2.setPower(n);
    }
}