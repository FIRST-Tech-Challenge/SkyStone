package org.firstinspires.ftc.teamcode.HardwareSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/* An extrusion class to automate all extrusion tasks
The direction of the motors must be such that a positive power causes the encoder to increase
*/

public class Extrusion {

    private int upperLimit;
    private int lowerLimit;
    private DigitalChannel limitSwitch;

    //private PID run;

    private double powerLowLimit;
    private double powerHighLimit;

    private LinearOpMode op;

    public Extrusion(int upLimit, int lowLimit, LinearOpMode oppy) {

        this.upperLimit = upLimit;
        this.lowerLimit = lowLimit;
        this.limitSwitch = null;
        this.op = oppy;

    }

    /*
    public void setPidConstants(double P, double I, double D) {
        //run = new PID(P, I, D, 15, 0.7, 0);
    }

    public void initialize(double minPower, double maxPower) {

        //lowSwitch.setMode(DigitalChannel.Mode.INPUT);


        this.powerLowLimit = minPower;
        this.powerHighLimit = maxPower;

    }

    // Utility Methods =============================================================================

    private void setPower(double power) {
        if(power > powerHighLimit) {
            power = powerHighLimit;
        }else if(power < -powerHighLimit) {
            power = -powerHighLimit;
        }

        //motor1.setPower(power);
        //motor2.setPower(power);

    }

    // Autonomous Methods ==========================================================================

    public void runToPosition(double target) {

        double correct;
        while(op.opModeIsActive()) {
            //correct = run.getCorrection(target, motor1.getCurrentPosition());
            if (Math.abs(correct) > powerLowLimit) {
                setPower(-correct);
            }else {
                break;
            }
        }
        isRunning = false;
    }

    public void extend(String method) {
        isRunning = true;
        if(method.equals("encoder")) {
            runToPosition(upperLimit - 10);
        }
        isRunning = false;
    }

    public void retract(String method) {
        isRunning = true;
        if(method.equals("encoder")) {
            runToPosition(lowerLimit);
        }else if(method.equals("switch")) {
            while(lowSwitch.getState()) {
                setPower(-0.4);
            }
        }
        isRunning = false;
    }

    // Continuous Methods ==========================================================================
    public void runManual(double input, double deadZone) {
        if(Math.abs(input) > deadZone) {
            setPower(input);
        }else {
            setPower(0);
        }
    }

    public void extrudeManual(boolean trigger) {
        if(trigger) {
            setPower(0.4);
        }else{
            setPower(0);
        }
    }

    public void retractManual(boolean trigger) {
        if(trigger) {
            setPower(-0.4);
        }else{
            setPower(0);
        }
    }

    public void runToPositionTeleOp(double target) {
        double correct = run.getCorrection(target, motor1.getCurrentPosition());
        if (Math.abs(correct) > powerLowLimit) {
            isRunning = true;
            setPower(correct);
        }else {
            setPower(0);
            isRunning = false;
        }
    }

    public void retractTeleOp() {
        if (!lowSwitch.getState()) {
            setPower(0);
            isRunning = false;
        }else {
            setPower(-0.4);
            isRunning = true;
        }
    }
     */
}