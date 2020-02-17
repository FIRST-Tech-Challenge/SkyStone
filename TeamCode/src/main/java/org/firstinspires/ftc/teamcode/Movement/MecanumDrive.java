package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

public class MecanumDrive extends Drivebase {

    public MecanumDrive(){}

    @Override
    public void initialize(){
        lf = 0;
        rf = 0;
        lb = 0;
        rb = 0;

        reverseMotors();
        setRunMode("withoutEncoder");
        setPowerBehavior("brake");

    }

    @Override
    public void update(){

        RobotHardware.rightFront.setPower(rf);
        RobotHardware.leftFront.setPower(lf);
        RobotHardware.leftBack.setPower(lb);
        RobotHardware.rightBack.setPower(rb);

    }

    public void setRelativeVelocity(double velX, double velY, double velHeading){
        /* Wheel movement to move horizontally
          A          |
          |          V
               -->
          |          A
          V          |
        */
        lf = velY + velX*1.1 - velHeading/2;
        rf = velY - velX*1.1 + velHeading/2;
        lb = velY - velX*1.1 - velHeading/2;
        rb = velY + velX*1.1 + velHeading/2;

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
            RobotHardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RobotHardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RobotHardware.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RobotHardware.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else if(behavior.equals("coast")){
            RobotHardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            RobotHardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            RobotHardware.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            RobotHardware.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    private void setRunMode(String runMode){

        if(runMode.equals("withEncoder")){
            RobotHardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotHardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotHardware.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotHardware.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(runMode.equals("withoutEncoder")){
            RobotHardware.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RobotHardware.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RobotHardware.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RobotHardware.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void resetDriveEncoders(){

        RobotHardware.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotHardware.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotHardware.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotHardware.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private void reverseMotors(){

        // Reverse the necessary motors so that when positive power is set to all four, the robot moves forward
        RobotHardware.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RobotHardware.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RobotHardware.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        RobotHardware.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }

}
