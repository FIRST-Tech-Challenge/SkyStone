package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Maccabot {

    // Pulling in OpMode data
    private OpMode parentOpMode;
    private HardwareMap hardwareMap;

    // Drive Motor Variables
    private DcMotor front_left, front_right, back_left, back_right;

    // Intake Motors TBD
    // private DcMotor intake_left, intake_right;

    public Maccabot(OpMode parentOpMode){
        this.parentOpMode = parentOpMode;
        this.hardwareMap = parentOpMode.hardwareMap;
    }

    public void initializeRobot(){
        parentOpMode.telemetry.addLine("Initializing Drive");
        // Get drive motors
        front_left = hardwareMap.dcMotor.get("front_left"); // Port 0
        front_right = hardwareMap.dcMotor.get("front_right"); // Port 1
        back_left = hardwareMap.dcMotor.get("back_left"); // Port 2
        back_right = hardwareMap.dcMotor.get("back_right"); // Port 3

        // Reverse Right Side
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO PID!!!
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drive Motors Shouldn't Drive
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double flPower, double frPower, double blPower, double brPower){
        front_left.setPower(flPower);
        front_right.setPower(frPower);
        back_left.setPower(blPower);
        back_right.setPower(brPower);
        // print encoder values
        parentOpMode.telemetry.addLine(Integer.toString(front_left.getCurrentPosition()));
        parentOpMode.telemetry.addLine(Integer.toString(front_right.getCurrentPosition()));
        parentOpMode.telemetry.addLine(Integer.toString(back_left.getCurrentPosition()));
        parentOpMode.telemetry.addLine(Integer.toString(back_right.getCurrentPosition()));
    }

    public void mecanumDrive(double vtX, double vtY, double vR){
        double flValue = vtY + vtX - vR;
        double frValue = vtY - vtX + vR;
        double blValue = vtY - vtX - vR;
        double brValue = vtY + vtX + vR;

        drive(flValue, frValue, blValue, brValue);
    }

}
