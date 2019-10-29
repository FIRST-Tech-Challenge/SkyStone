package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Base64;

@Autonomous(name="Odometry Auto", group="Linear Opmode")
public class OdometryAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    // defining back right wheel
    private DcMotor left1;
    // defining back left wheel
    private DcMotor left2;
    // defining front right wheel
    private DcMotor right1;
    // defining back left wheel
    private DcMotor right2;
    // color sensor

//    private ColorSensor color;
//    // only for encoder use
//
//    private DcMotor leftOdo;
//    // only for encoder use
//    private DcMotor rightOdo;
//    // only for encoder use
//    private DcMotor alignOdo;
//    // only for encoder alignment
//
//    private DcMotor intakeLeft;
//    // only for motor intake left motor
//    private DcMotor intakeRight;
//    // only for motor outtake right motor
//    private Servo outtakeLeft;
//    // only for motor outtake left
//    private Servo outtakeRight;
//    // only for motor outtake right
//
//    private DcMotor elevatorRight;
//    // only for elevator right motor
//    private DcMotor elevatorLeft;
//    // only for elevator left motor
//
//    private Servo grabRight;
//    // only for grabber right
//    private Servo grabLeft;
//    // only for grabber left
//
//    private Servo nubGrabRight;
//    // only for the nub grabber right
//    private Servo nubGrabLeft;
//    // only for the nub grabber left


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("leftFront");
        left2 = hardwareMap.dcMotor.get("leftBack");
        right1 = hardwareMap.dcMotor.get("rightFront");
        right2 = hardwareMap.dcMotor.get("rightBack");

//        color = hardwareMap.colorSensor.get("color");
//
//        leftOdo = hardwareMap.dcMotor.get("leftOdo");
//        rightOdo = hardwareMap.dcMotor.get("rightOdo");
//        alignOdo = hardwareMap.dcMotor.get("alignOdo");
//
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
//        nubGrabRight = hardwareMap.servo.get("nubGrabright");


        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        telemetry.addData("leftodo", left1.getCurrentPosition());
        telemetry.update();


//        straightMovement(12, 1, 1, 1, 1);
//        sideToSide(12, 1, -1, -1, 1);
    }
//
//    int numOfTicks = 4096;
//    double pi = 3.14159265358979323;
//    int wheelDiameter = 2;
//    double conversion = numOfTicks / (wheelDiameter * pi);
//
//    void straightMovement(int inches, int L1, int L2, int R1, int R2) {
//
//        int currentPosLeft = leftOdo.getCurrentPosition();
//        // insert name of left odometry wheel
//        int currentPosRight = rightOdo.getCurrentPosition();
//        // insert name of the right odometry wheel
//        int currentAlignment = alignOdo.getCurrentPosition();
//
//        double ticksMovement = conversion * inches;
//
//        double targetPosLeft = currentPosLeft + ticksMovement;
//        double targetPosRight = currentPosRight + ticksMovement;
//
//        if (currentPosLeft != targetPosLeft && currentPosRight != targetPosRight) {
//            left1.setPower(L1);
//            left2.setPower(L2);
//            right1.setPower(R1);
//            right2.setPower(R2);
//        } else {
//            left1.setPower(0);
//            left2.setPower(0);
//            right1.setPower(0);
//            right2.setPower(0);
//        }
//        alignOdo.setTargetPosition(currentAlignment);
//    }
//
//    void sideToSide(int inches, int L1, int L2, int R1, int R2) {
//
//        int currentPosLeft = leftOdo.getCurrentPosition();
//        // insert name of left odometry wheel
//        int currentPosRight = rightOdo.getCurrentPosition();
//        // insert name of the right odometry wheel
//        int currentAlignment = alignOdo.getCurrentPosition();
//
//        double ticksMovement = conversion * inches;
//
//        double targetAlignPos = currentAlignment + ticksMovement;
//
//        if (currentAlignment != targetAlignPos) {
//            left1.setPower(L1);
//            left2.setPower(L2);
//            right1.setPower(R1);
//            right2.setPower(R2);
//        } else {
//            left1.setPower(0);
//            left2.setPower(0);
//            right1.setPower(0);
//            right2.setPower(0);
//        }
//        leftOdo.setTargetPosition(currentPosLeft);
//        rightOdo.setTargetPosition(currentPosRight);
//    }
}