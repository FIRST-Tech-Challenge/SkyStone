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

    private DcMotor elevatorRight;
    // only for elevator right motor
    private DcMotor elevatorLeft;
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

        left1 = hardwareMap.dcMotor.get("leftFront");
        left2 = hardwareMap.dcMotor.get("leftBack");
        right1 = hardwareMap.dcMotor.get("rightFront");
        right2 = hardwareMap.dcMotor.get("rightBack");

//        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
//        intakeRight = hardwareMap.dcMotor.get("intakeRight");
//        outtakeLeft = hardwareMap.servo.get("outtakeLeft");
//        outtakeRight = hardwareMap.servo.get("outtakeRight");
//
//        elevatorLeft = hardwareMap.dcMotor.get("elevatorLeft");
//        elevatorRight = hardwareMap.dcMotor.get("elevatorRight");
//
//        grabRight = hardwareMap.servo.get("grabRight");
//        grabLeft = hardwareMap.servo.get("grabLeft");
//
//        nubGrabLeft = hardwareMap.servo.get("nubGrabLeft");
//        nubGrabRight = hardwareMap.servo.get("nubGrabRight");

        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

//        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.addData("leftOdo", left2.getCurrentPosition());
            telemetry.addData("nothing", left1.getCurrentPosition());
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


//            if (gamepad2.left_bumper) {
//                telemetry.addData("outtakeRight", outtakeRight);
//                telemetry.addData("outtakeLeft", outtakeLeft);
//            }
//
//            if (gamepad2.right_bumper) {
//                intakeLeft.setPower(1);
//                intakeRight.setPower(1);
//                telemetry.addData("intakeLeft", intakeLeft);
//                telemetry.addData("intakeRight", intakeRight);
//            }
//
//            if (gamepad2.dpad_up) {
//                telemetry.addData("elevator right", elevatorRight);
//                telemetry.addData("elevator left", elevatorLeft);
//                telemetry.update();
//            }
//
//            if (gamepad2.dpad_down) {
//                telemetry.addData("elevator right", elevatorRight);
//                telemetry.addData("elevator left", elevatorLeft);
//                telemetry.update();
//            }
//            if (gamepad1.a) {
//                telemetry.addData("nub grabber right", nubGrabRight);
//                telemetry.addData("nub grabber left", nubGrabLeft);
//                telemetry.update();
//            }
//            if (gamepad2.b) {
//                telemetry.addData("grabber right", grabRight);
//                telemetry.addData("grabber left", grabLeft);
//                telemetry.update();
//            }
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