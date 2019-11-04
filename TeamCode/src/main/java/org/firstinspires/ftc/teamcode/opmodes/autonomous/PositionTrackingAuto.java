package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Math.sqrt;

@Autonomous(name="Position Tracking", group="Linear Opmode")
public class PositionTrackingAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    // defining front left wheel
    private DcMotor left1;
    // defining back left wheel
    private DcMotor left2;
    // defining front right wheel
    private DcMotor right1;
    // defining back right wheel
    private DcMotor right2;

    private DcMotor leftOdo;
    // only for encoder use
    private DcMotor rightOdo;
    // only for encoder use
    private DcMotor alignOdo;
    // only for encoder alignment

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("left1");
        left2 = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        int leftOdo = left1.getCurrentPosition();
        int rightOdo = left2.getCurrentPosition();
        int alignOdo = right1.getCurrentPosition();

        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        telemetry.addData("leftodo", leftOdo);
        telemetry.addData("rightodo", rightOdo);
        telemetry.addData("alignodo", alignOdo);
        telemetry.update();
    }

    int numOfTicks = 4096;
    double pi = 3.14159265358979323;
    int wheelDiameter = 2;
    double conversion = numOfTicks / (wheelDiameter * pi);

    void odomTutorial (int x, int y){
        int initialX = 0;
        int initialY = 0;
        int initialTheta = 0;

        double diagonalMovement = sqrt((x * x) + (y * y));
        double ticksMovement = diagonalMovement * conversion;

        int currentLeftPos = leftOdo.getCurrentPosition();
        int currentRightPos = rightOdo.getCurrentPosition();
        int currentAlignPos = alignOdo.getCurrentPosition();

        double targetLeftPos = ticksMovement + currentLeftPos;
        double targetRightPos = ticksMovement + currentRightPos;

        if (leftOdo.getCurrentPosition() != targetLeftPos && rightOdo.getCurrentPosition() != targetRightPos) {
            left2.setPower(1);
            right1.setPower(1);
        }
        else {
            left2.setPower(0);
            right1.setPower(0);
        }


    }

//    void moveToPosition(int x, int y, int angle, int turn) {
//        double diagonalMovement = sqrt((x * x) + (y * y));
//        double ticksMovement = diagonalMovement * conversion;
//
//        int currentLeftPos = leftOdo.getCurrentPosition();
//        int currentRightPos = rightOdo.getCurrentPosition();
//        int curentAlignPos = alignOdo.getCurrentPosition();
//
//        double targetLeftPos = ticksMovement + currentLeftPos;
//        double targetRightPos = ticksMovement + currentRightPos;
//
//
//
//        if (angle < 90 && angle > 0) {
//            while (currentLeftPos != leftOdo.getCurrentPosition()) {
//                left1.setPower(1);
//                right2.setPower(1);
//            }
//        }
//        if (angle < 180 && angle > 90) {
//            while (currentLeftPos != leftOdo.getCurrentPosition()) {
//                left2.setPower(1);
//                right1.setPower(1);
//            }
//        }
//        if (angle < 0 && angle > -90) {
//            while (currentLeftPos != leftOdo.getCurrentPosition()) {
//                left2.setPower(-1);
//                right1.setPower(-1);
//            }
//        }
//        if (angle < -90 && angle > -180) {
//            while (currentLeftPos != leftOdo.getCurrentPosition()) {
//                left1.setPower(-1);
//                right2.setPower(-1);
//            }
//        }
//    }
}
