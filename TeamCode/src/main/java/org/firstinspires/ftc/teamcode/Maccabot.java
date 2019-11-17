package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@TeleOp(name="MaccaBot", group="Drive Systems")
public class Maccabot extends LinearOpMode {

    // Pulling in OpMode data
    private OpMode parentOpMode;
    private HardwareMap hardwareMap;

    // Drive Motor Variables
    private DcMotor front_left, front_right, back_left, back_right; //intake_left, intake_right

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
        //intake_left = hardwareMap.dcMotor.get("intake_left");
        //intake_right = hardwareMap.dcMotor.get("intake_right");


        // Reverse Right Side
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        //intake_left.setDirection(DcMotorSimple.Direction.REVERSE);

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
        //intake_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    /*public void intake(double power, int direction){
        intake_right.setPower(power * direction);
        intake_left.setPower(power * direction);

        parentOpMode.telemetry.addLine(Integer.toString(intake_right.getCurrentPosition()));
        parentOpMode.telemetry.addLine(Integer.toString(intake_left.getCurrentPosition()));
    }*/

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
