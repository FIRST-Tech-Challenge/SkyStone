package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

/**
 * hi iit is Eliezer in 09; Dis now Jacob 18
 * Douglas Edit
 * Owen Edit (Owen is Nerd though)
 */

@TeleOp(name="Part test", group="Test")
public class MainTeleOp extends OpMode{//cool bearl real cool

    /*
    Part declarations.
    Add parts here as we progress
    */

    // Top left, top right, bottom left, bottom right, arm motors
    private DcMotor tl, tr, bl, br, arm, lift, n1;
    private CRServo intake1, intake2, intake3, intake4; //1 is tr, 2 is tl, 3 is br, 4 is bl
    private float   leftPower, rightPower, xValue, yValue;


    // Code to run after init is hit
    public void init(){

        /*
        Register all parts. Note that the string passed
        is what the phone looks for in its configuration
        to map software motors to their actual hardware motor
        */
        tl = hardwareMap.dcMotor.get("top_left_wheel");
        tr = hardwareMap.dcMotor.get("top_right_wheel");
        bl = hardwareMap.dcMotor.get("bottom_left_wheel");
        br = hardwareMap.dcMotor.get("bottom_right_wheel");
        arm = hardwareMap.dcMotor.get("arm");
        n1 = hardwareMap.dcMotor.get("n1");
        intake1 = hardwareMap.crservo.get("intake1");
        intake2 = hardwareMap.crservo.get("intake2");
        intake3 = hardwareMap.crservo.get("intake3");
        intake4 = hardwareMap.crservo.get("intake4");
        lift = hardwareMap.dcMotor.get("lift");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//encoder standards for the arm
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);


        // Note: motors spin clockwise by default


    }

    // Runs given motor at 1 when cond is true


    //top left and top right switch
    private void testMotor(boolean cond, DcMotor dc, DcMotor dc2, DcMotor dc3, DcMotor dc4 ) {
        if(cond) { dc.setPower(-1); } else { dc.setPower(0); }  //changed from 1 to -1
        if(cond) { dc2.setPower(-1); } else { dc2.setPower(0); }
        if(cond) { dc3.setPower(-1); } else { dc3.setPower(0); } //no change
        if(cond) { dc4.setPower(-1); } else { dc4.setPower(0); } //no change
    }
    private void testMotorback(boolean cond, DcMotor dc, DcMotor dc2, DcMotor dc3, DcMotor dc4 ) {
        if(cond) { dc.setPower(1); } else { dc.setPower(0); }   //changed from -1 to 1
        if(cond) { dc2.setPower(1); } else { dc2.setPower(0); } //changed from -1 to 1
        if(cond) { dc3.setPower(1); } else { dc3.setPower(0); }    //no change
        if(cond) { dc4.setPower(1); } else { dc4.setPower(0); }    //no change
    }

    private void testMotorright(double cond, DcMotor dc, DcMotor dc2, DcMotor dc3, DcMotor dc4 ) {

        tl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        tr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        if(cond > 0) { dc.setPower(0.75); } else { dc.setPower(0); }   //changed from 1 to -1
        if(cond > 0) { dc2.setPower(-0.75); } else { dc2.setPower(0); }    // no change
        if(cond > 0) { dc3.setPower(-0.75); } else { dc3.setPower(0); } //changed from 1 to -1
        if(cond > 0) { dc4.setPower(0.75); } else { dc4.setPower(0); } // no change
    }
    private void testMotorleft(double cond, DcMotor dc, DcMotor dc2, DcMotor dc3, DcMotor dc4 ) {

        tl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        tr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);


        if(cond < 0) { dc.setPower(-0.75); } else { dc.setPower(0); }  //no change
        if(cond < 0) { dc2.setPower(0.75); } else { dc2.setPower(0); }    //changed from 1 to -1
        if(cond < 0) { dc3.setPower(0.75); } else { dc3.setPower(0); }    //no change
        if(cond < 0) { dc4.setPower(-0.75); } else { dc4.setPower(0); }    //changed from 1 to -1
    }
    private void strafe(double cond, DcMotor dc, DcMotor dc2, DcMotor dc3, DcMotor dc4){//code to make the robot strafe
        if(cond > 0) { dc.setPower(1); } else {dc.setPower(0);} //right
        if(cond > 0) { dc2.setPower(-1); } else {dc2.setPower(0);}
        if(cond > 0) { dc3.setPower(1);} else {dc3.setPower(0);}
        if(cond > 0) { dc4.setPower(-1); } else {dc2.setPower(0);}
        if(cond < 0) { dc.setPower(-1); } else {dc.setPower(0);} // left
        if(cond < 0) { dc2.setPower(1); } else {dc2.setPower(0);}
        if(cond < 0) { dc3.setPower(-1);} else {dc3.setPower(0);}
        if(cond < 0) { dc4.setPower(1); } else {dc2.setPower(0);}

    }


    // Runs repeatedly
    public void loop(){
        // Check if buttons are being pressed and run motors

        reboundMecanumDrive(-(gamepad1.left_stick_x), gamepad1.left_stick_y, gamepad1.right_stick_x); //left right, forwards backwards, turning

        /*yValue = gamepad1.left_stick_y;
        xValue = gamepad1.left_stick_x;

        tl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        tr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPower =  yValue - xValue;
        rightPower = yValue + xValue;
            tl.setPower(Range.clip(leftPower, -0.75, 0.75));
            bl.setPower(Range.clip(leftPower, -0.75, 0.75));
            tr.setPower(Range.clip(rightPower, -0.75, 0.75));
            br.setPower(Range.clip(rightPower, -0.75, 0.75));*/



        telemetry.addData("Mode", "running");//these are the encoder values
        telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
        telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
        telemetry.addData("Jordan Kramer", "Is Cool");
        telemetry.update();

        testMotorright(gamepad1.right_stick_x, tl, tr, bl, br);
        testMotorleft(gamepad1.right_stick_x, tl, tr, bl, br);
        //this is what establishes the motors in the controller, encoder, and phones "mind"


        /*double power;
        power    = Range.clip(-gamepad1.right_stick_y, -0.6, 0.6) ;//change min between -1 and 0, max between 0 and 1 to make slower/faster
        arm.setPower(power);*/

        if(gamepad2.dpad_up){arm.setPower(-1);}
        if(gamepad2.dpad_down){arm.setPower(1);}

        if(!(gamepad2.dpad_up || gamepad2.dpad_down)){arm.setPower(0);}

        // Check if buttons are being pressed and run servos
        if(gamepad2.a){
            intake1.setPower(1);
            intake2.setPower(-1);
        }
        else{if(!(gamepad2.right_bumper || gamepad2.left_bumper || gamepad2.a || gamepad2.b )){ intake1.setPower(0); intake2.setPower(0);}}

        if(gamepad2.b){// Check if buttons are being pressed and run servos
            intake3.setPower(1);
            intake4.setPower(-1);
        }
        else{if(!(gamepad2.right_bumper || gamepad2.left_bumper || gamepad2.a || gamepad2.b )){ intake3.setPower(0); intake4.setPower(0);}}

        if(gamepad2.right_bumper){// Check if buttons are being pressed and run servos
            intake1.setPower(-1);
            intake2.setPower(1);
        }
        else{if(!(gamepad2.right_bumper || gamepad2.left_bumper || gamepad2.a || gamepad2.b )){ intake1.setPower(0); intake2.setPower(0);}}

        if(gamepad2.left_bumper){// Check if buttons are being pressed and run servos
            intake3.setPower(-1);
            intake4.setPower(1);
        }
        else{if(!(gamepad2.right_bumper || gamepad2.left_bumper || gamepad2.a || gamepad2.b )){ intake3.setPower(0); intake4.setPower(0);}}//// Check if buttons are being pressed and sets the power to 0

        if(gamepad2.right_trigger > 0){lift.setPower(gamepad2.right_trigger);}
        else{if(gamepad2.left_trigger > 0){lift.setPower(-gamepad2.left_trigger);}
        else{lift.setPower(0);}}

        if(gamepad2.x){// Check if directional button is being used for arm power,if not it set the base height to .0018, which is just above the ground
            if (arm.getCurrentPosition() < 0){
                pArmToLanderFromGround(0.0018, 0, arm);
            }
        } //automatically brings arm up un lander position

        telemetry.addLine(String.valueOf(lift.getCurrentPosition()));//gets the values of the arms position from the encoder.

        if(gamepad1.x){n1.setPower(-1+Math.random()*2);}
        if(gamepad1.a){n1.setPower(1);}
        if(gamepad1.b){n1.setPower(-1);}

    }
    private void pArmToLanderFromGround(double kP, double target, DcMotor driveMotor) {
        double error = Math.abs(target - driveMotor.getCurrentPosition());//obtains the arm's position
        while (error > 1) {//allows the robot to continually operate
            driveMotor.setPower(kP * error);
            error = Math.abs(target - driveMotor.getCurrentPosition());
        }
        arm.setPower(0);
    }
    public void reboundMecanumDrive(double vtX, double vtY, double vR) {
        // calculate motor powers
        double tlPower = vtY + vtX - vR;
        double trPower = vtY - vtX + vR;
        double blPower = -(vtY) - vtX - vR;
        double brPower = -(vtY) + vtX + vR;
        // set motor powers
        tl.setPower(-0.8*tlPower);
        tr.setPower(-0.8*trPower);
        bl.setPower(0.8*blPower);
        br.setPower(0.8*brPower);
    }
}
