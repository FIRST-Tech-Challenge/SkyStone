package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Encoder Auto", group="Linear Opmode")
public class EncoderAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;

    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    private DcMotor liftRight;
    // only for elevator right motor
    private DcMotor liftLeft;
    // only for elevator left motor


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

        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        moveForwardInchesAtPower(12,1);
    }

    public void setLeftRightPower(double leftPower, double rightPower){
        setLeftPower(leftPower);
        setRightPower(rightPower);
    }

    public void setLeftPower(double leftPower){
        left1.setPower(leftPower);
        left2.setPower(leftPower);
    }

    public void setRightPower(double rightPower){
        right1.setPower(rightPower);
        right2.setPower(rightPower);
    }

    public final double WHEEL_DIAMETER = 4;
    public final double TICKS_PER_REVOLUTION = 1120;
    public final double PI = 3.141592653589793;

    public void moveForwardInchesAtPower(double inputInchesForward, double inputSpeed){
        int initialRightMotorPosition = right1.getCurrentPosition();
        int initialLeftMotorPosition  = left1.getCurrentPosition();
        int ticksToMove = (int) (TICKS_PER_REVOLUTION/(PI * WHEEL_DIAMETER) * inputInchesForward);
        int targetRightMotorPosition = initialRightMotorPosition + ticksToMove;
        int targetLeftMotorPosition  = initialLeftMotorPosition  + ticksToMove;
        boolean isInLeftRange = true;
        boolean isInRightRange = true;
        while(isInLeftRange || isInRightRange){
            if(isInRightRange){
                setRightPower(inputSpeed);
            }
            else{
                setRightPower(0);
            }
            if(isInLeftRange){
                setLeftPower(inputSpeed);
            }
            else{
                setLeftPower(0);
            }
            isInRightRange = right1.getCurrentPosition() < targetRightMotorPosition;
            isInLeftRange  = left1.getCurrentPosition() < targetLeftMotorPosition;
        }
        setLeftRightPower(0, 0);
    }
}

