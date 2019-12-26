package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Maccabot {

    // Pulling in OpMode data
    private OpMode parentOpMode;
    private HardwareMap hardwareMap;
    private double encoder;

    // Drive Motor Variables
    private DcMotor front_left, front_right, back_left, back_right, intake_left, intake_right, lift_left, lift_right;
    private Servo servo;
    private CRServo claw;

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
        intake_left = hardwareMap.dcMotor.get("intake_left");
        intake_right = hardwareMap.dcMotor.get("intake_right");
        lift_left = hardwareMap.dcMotor.get("lift_left");
        lift_right = hardwareMap.dcMotor.get("lift_right");
        servo = hardwareMap.servo.get("servo");
        claw = hardwareMap.crservo.get("bob");

        encoder = 0;




        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        intake_left.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO PID!!!
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Drive Motors Shouldn't Drive
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double flPower, double frPower, double blPower, double brPower){
        front_left.setPower(flPower);
        front_right.setPower(frPower);
        back_left.setPower(blPower);
        back_right.setPower(brPower);
        // print encoder values
        /*parentOpMode.telemetry.addLine(Integer.toString(front_left.getCurrentPosition()));
        parentOpMode.telemetry.addLine(Integer.toString(front_right.getCurrentPosition()));
        parentOpMode.telemetry.addLine(Integer.toString(back_left.getCurrentPosition()));
        parentOpMode.telemetry.addLine(Integer.toString(back_right.getCurrentPosition()));*/
    }

    public void mecanumDrive(double vtX, double vtY, double vR){
        double flValue = vtY + vtX - vR;
        double frValue = vtY - vtX + vR;
        double blValue = vtY - vtX - vR;
        double brValue = vtY + vtX + vR;

        drive(flValue, frValue, blValue, brValue);
    }

    public void intake(double cond1) {
        intake_right.setPower((cond1)*.5);//the multiplication of a decimal reduces the motor speed for the INTAKE
        intake_left.setPower((cond1)*.5);

       /* parentOpMode.telemetry.addLine(Double.toString(intake_right.getPower()));
        parentOpMode.telemetry.addLine(Double.toString(intake_left.getPower()));*/
    }

    public void servo( double cond1){
        claw.setPower(cond1);
    }

    public void lift(double cond1, double cond2){

        lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if((lift_right.getCurrentPosition() <= 0 && (cond2 > 0)) || (lift_right.getCurrentPosition() >= 2030 && (cond1 > 0))){
            lift_right.setPower(0);
            lift_left.setPower(0);
            lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else{
            lift_left.setPower((cond1-cond2) * 0.75);//the multiplication of a decimal reduces the motor speed for the LIFT
            lift_right.setPower((cond1-cond2) * 0.75);

            parentOpMode.telemetry.addLine(Double.toString(lift_right.getCurrentPosition()));
            parentOpMode.telemetry.update();
        }

    }

    public void setpower0(){
        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }

    public void auto_forward(int pos, double power){//method for autored for the robot to go forward
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        back_left.setPower(power);
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(power);
        front_left.setTargetPosition(-pos);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(power);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_right.setPower(power);
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();


    }

    public void auto_backward(int pos, double power){//method for autored for the robot to go backward
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        back_left.setPower(-power);//inverted direction for all the motors, so watch out, it goes backwards
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(-power);
        front_left.setTargetPosition(-pos);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(-power);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_right.setPower(-power);
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();


    }
    public void auto_turnright(int pos, double power){//autored method for turn right
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        back_left.setPower(power);
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(power);
        front_left.setTargetPosition(-pos);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(-power);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);//negative power for the right side so the robot does a pinpoint turn to the right

        back_right.setPower(-power);
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();

    }
    public void auto_turnleft(int pos, double power){//autored method for turning left
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        back_left.setPower(-power);//mirror of turn right, but the left motors are negative to pinpoint turn left
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(-power);
        front_left.setTargetPosition(-pos);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(power);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_right.setPower(power);
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();

    }
    public void auto_straferight(int pos, double power){//method for strafing right
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        back_left.setPower(power);
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(-power);
        front_left.setTargetPosition(-pos);//front left inverted to allow for strafing
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(power);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_right.setPower(-power);//back left inverted for strafing
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();
    }
    public void auto_strafeleft(int pos, double power){//mirror of strafe right except with back left and front right inverted for a left strafe.
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        

        back_left.setPower(-power);
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(power);
        front_left.setTargetPosition(-pos);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(-power);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_right.setPower(power);
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();
    }

    public boolean encoderIsBusy() {
        return (front_right.isBusy());
    }
    public void resetEncoder(){
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runwithoutencoder(){
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void withoutencoder_strafe_left(double power){
        runwithoutencoder();
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        back_left.setPower(-power);
        front_left.setPower(power);
        front_right.setPower(-power);
        back_right.setPower(power);

    }

}
