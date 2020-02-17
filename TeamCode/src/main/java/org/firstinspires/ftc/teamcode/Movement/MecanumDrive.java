package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

public class MecanumDrive extends Drivebase {

    public MecanumDrive(RobotHardware robotHardware){

        this.rightFront = robotHardware.rightFront;
        this.leftFront = robotHardware.leftFront;
        this.leftBack = robotHardware.leftBack;
        this.rightBack = robotHardware.rightBack;

    }

    @Override
    public void initialize(){
        setRunMode("withEncoder");
        setPowerBehavior("coast");
        resetDriveEncoders();
        reverseMotors();

    }

    @Override
    public void update(){

        rightFront.setPower(rf);
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);

    }

    public void setRelativeVelocity(double velX, double velY, double velHeading){
        /* Wheel movement to move horizontally
          A          |
          |          V
               -->
          |          A
          V          |
        */
        lf = velY + velX*1.3 - velHeading/2;
        rf = velY - velX*1.3 + velHeading/2;
        lb = velY - velX*1.3 - velHeading/2;
        rb = velY + velX*1.3 + velHeading/2;

    }

    @Override
    public void freeze(){

        lf = 0;
        rf = 0;
        lb = 0;
        rb = 0;
        update();

    }

    private void setPowerBehavior(String behavior){

        if(behavior.equals("brake")){
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else if(behavior.equals("coast")){
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    private void setRunMode(String runMode){

        if(runMode.equals("withEncoder")){
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(runMode.equals("withoutEncoder")){
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void resetDriveEncoders(){

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private void reverseMotors(){

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }

}
