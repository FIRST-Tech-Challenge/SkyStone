package org.firstinspires.ftc.teamcode.Autonomous.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;
import org.firstinspires.ftc.teamcode.Tensorflow.TFODCalc;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.PI;

public class Align {
    HardwareMap hwMap;
    private double approachingPower = 0.5;
    private double turnPower = 0.4;
    private LinearOpMode opMode;
    private List<Recognition> detectedObj;
    private boolean atOriginalPos = false;
    private double externalHeading = -1;

    public Align(HardwareMap hwMap, LinearOpMode opMode, DcMotor.ZeroPowerBehavior zeroPower) {
        this.hwMap = hwMap;
        this.opMode = opMode;
        hwMap.frontRight.setZeroPowerBehavior(zeroPower);
        hwMap.frontLeft.setZeroPowerBehavior(zeroPower);
        hwMap.backRight.setZeroPowerBehavior(zeroPower);
        hwMap.backLeft.setZeroPowerBehavior(zeroPower);
    }

    public void setPower(double approachingPower, double turnPower){
        this.approachingPower = approachingPower;
        this.turnPower = turnPower;
    }

    public void foundation(){
        double delta90 = Math.abs(450 - externalHeading);
        double delta270 = Math.abs(externalHeading - 270);
        boolean correctRotation = false;

        while(!opMode.isStopRequested() && opMode.opModeIsActive()) {
            if (delta270 <= delta90 && !correctRotation) {
                if (externalHeading >= 280) {
                    setRightPower(-turnPower);
                    setLeftPower(turnPower);
                } else if (externalHeading <= 260) {
                    setRightPower(turnPower);
                    setLeftPower(-turnPower);
                }
            } else if (delta270 > delta90 && !correctRotation) {
                if (externalHeading <= 80) {
                    setRightPower(turnPower);
                    setLeftPower(-turnPower);
                } else if (externalHeading >= 100) {
                    setRightPower(-turnPower);
                    setLeftPower(turnPower);
                }
            }

            if ((externalHeading > 80 && externalHeading < 100) || (externalHeading < 280 && externalHeading > 260) &&
                    !correctRotation) {
                stop();
                correctRotation = true;
            }

            if(correctRotation && !hwMap.foundationDetect1.getState() && !hwMap.foundationDetect2.getState()){
                setPower(-approachingPower);
            } else if(correctRotation && hwMap.foundationDetect1.getState() && !hwMap.foundationDetect2.getState()){
                strafe(approachingPower, false);
            } else if(correctRotation && !hwMap.foundationDetect1.getState() && hwMap.foundationDetect2.getState()){
                strafe(approachingPower, true);
            } else if(correctRotation && hwMap.foundationDetect1.getState() && hwMap.foundationDetect2.getState()){
                stop();
            }
        }

    }

    public void skystone() {
        double horizontalMid = -1;
        double distanceAway = -1;
        double theta = -1;
        double initLeft = getOdometry()[0];
        double initRight = getOdometry()[1];
        double initSide = getOdometry()[2];
        boolean isAligned = false;
        boolean collected = false;
        int imgWidth = -1;
        atOriginalPos = false;

        while (detectedObj != null) {
            // step through the list of recognitions and display boundary info.
            for (Recognition recognition : detectedObj) {
                if (recognition.getLabel().equalsIgnoreCase("skystone")) {
                    horizontalMid = getMiddleXY(recognition)[0];
                    distanceAway = TFODCalc.getDistanceToObj(127,
                            recognition.getImageHeight(), recognition.getHeight());
                    theta = TFODCalc.getAngleOfStone(recognition.getWidth(), distanceAway).get(0);
                    imgWidth = recognition.getImageWidth();

                    if (!isAligned && !opMode.isStopRequested()) {
                        if (horizontalMid >= imgWidth / 2d - 75 && horizontalMid <= imgWidth / 2d + 75) {
                            setPower(approachingPower);
                        } else if (horizontalMid <= imgWidth / 2d - 75) {
                            stop();
                            setRightPower(turnPower);
                        } else if (horizontalMid >= imgWidth / 2d + 75) {
                            stop();
                            setLeftPower(turnPower);
                        }

                        //if(distanceAway < 12 && theta > 35)
                        //    setPower(-approachingPower);

                        if (distanceAway < 12) {
                            stop();
                            isAligned = true;
                        }
                    }

                    intake(TeleopConstants.intakePower);

                    double left = getOdometry()[0];
                    double right = getOdometry()[1];

                    if (!collected && isAligned && !opMode.isStopRequested()) {
                        double averageDelta = (getOdometry()[0] - left) + (getOdometry()[1] - right) / 2;
                        double inchesMoved = averageDelta / 1400d * 2 * PI * 2;
                        //collected = intook();

                        if (inchesMoved > 12 /*|| collected*/) {
                            intake(0);
                            stop();
                            collected = true;
                        }
                    }

                    if (!atOriginalPos && collected && !opMode.isStopRequested()) {
                        double deltaL = getOdometry()[0] - initLeft;
                        double deltaR = getOdometry()[1] - initRight;

                        if (deltaL > 250)
                            setLeftPower(-approachingPower);
                        else if (deltaL < -250)
                            setLeftPower(approachingPower);

                        if (deltaR > 250)
                            setRightPower(-approachingPower);
                        else if (deltaR < -250)
                            setRightPower(approachingPower);

                        double deltaS = getOdometry()[2] - initSide;

                        if (deltaL <= 250 && deltaL >= -250 && deltaR <= 250 && deltaR >= -250) {
                            if (deltaS > 250)
                                strafe(approachingPower, true);
                            else if(deltaS < -250)
                                strafe(approachingPower, false);

                            if(deltaS <= 250 && deltaS >= -250) {
                                stop();
                                atOriginalPos = true;
                            }
                        }

                    }
                }
            }
        }
    }

    public void updateTFOD(List<Recognition> detectedObj){
        this.detectedObj = detectedObj;
    }

    public void updateExternalHeading(double externalHeading){
        this.externalHeading = externalHeading;
    }

    private double[] getMiddleXY(Recognition obj) {
        double horizontalMid = obj.getLeft() + obj.getWidth() / 2;
        double verticalMid = obj.getTop() + obj.getHeight() / 2;
        return new double[]{horizontalMid, verticalMid};
    }

    private void setPower(double power) {
        hwMap.frontRight.setPower(power);
        hwMap.frontLeft.setPower(power);
        hwMap.backRight.setPower(power);
        hwMap.backLeft.setPower(power);
    }

    private void stop(){
        hwMap.frontRight.setPower(0);
        hwMap.frontLeft.setPower(0);
        hwMap.backRight.setPower(0);
        hwMap.backLeft.setPower(0);
    }

    private void setLeftPower(double power) {
        hwMap.frontLeft.setPower(power);
        hwMap.backLeft.setPower(power);
    }

    private void setRightPower(double power) {
        hwMap.frontRight.setPower(power);
        hwMap.backRight.setPower(power);
    }

    private double[] getOdometry() {
        return new double[]{hwMap.leftIntake.getCurrentPosition(), hwMap.liftTwo.getCurrentPosition(),
                hwMap.rightIntake.getCurrentPosition()};
    }

    private void intake(double power) {
        hwMap.leftIntake.setPower(-power);
        hwMap.rightIntake.setPower(power);
    }

    private void strafe(double power, boolean right){
        if(!right)
            power = -power;

        hwMap.frontRight.setPower(-power);
        hwMap.frontLeft.setPower(power);
        hwMap.backRight.setPower(power);
        hwMap.backLeft.setPower(-power);
    }
}
