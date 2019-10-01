package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Mecanum Auto", group="Linear Opmode")
public class MecanumAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    // defining front left wheel
    private DcMotor left1;
    // defining back left wheel
    private DcMotor left2;
    // defining front right wheel
    private DcMotor right1;
    // defining back left wheel
    private DcMotor right2;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("left1");
        left2 = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        moveFront(1500);
        moveBack(1500);
        breakTime(2000);
        moveLeft(1500);
        moveRight(1500);
        breakTime(2000);
        northEast(1500);
        southWest(1500);
        breakTime(2000);
        northWest(1500);
        southEast(1500);
        breakTime(2000);
    }

        void moveFront(int sec) {
            left1.setPower(.5);
            left2.setPower(.5);
            right1.setPower(.5);
            right2.setPower(.65);
            sleep(sec);
        }

        void moveBack(int sec) {
            left1.setPower(-.5);
            left2.setPower(-.5);
            right1.setPower(-.5);
            right2.setPower(-.65);
            sleep(sec);
        }

        void moveLeft(int sec) {
            left1.setPower(-.5);
            left2.setPower(.5);
            right1.setPower(.5);
            right2.setPower(-.5);
            sleep(sec);
        }

        void moveRight(int sec) {
            left1.setPower(.5);
            left2.setPower(-.5);
            right1.setPower(-.5);
            right2.setPower(.5);
            sleep(sec);
        }

        void northEast(int sec) {
            left1.setPower(.5);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(.5);
            sleep(sec);
        }
        void northWest(int sec) {
            left1.setPower(0);
            left2.setPower(.5);
            right1.setPower(.5);
            right2.setPower(0);
            sleep(sec);
        }
        void southWest(int sec) {
            left1.setPower(-.5);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(-.5);
            sleep(sec);
        }
        void southEast(int sec) {
            left1.setPower(0);
            left2.setPower(-.5);
            right1.setPower(-.5);
            right2.setPower(0);
            sleep(sec);
        }

        void breakTime(int sec) {
            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);
            sleep(sec);
    }
}
