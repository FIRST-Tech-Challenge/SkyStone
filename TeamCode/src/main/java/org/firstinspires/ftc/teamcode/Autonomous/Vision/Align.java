package org.firstinspires.ftc.teamcode.Autonomous.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;
import org.firstinspires.ftc.teamcode.Tensorflow.TFODCalc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import javax.xml.transform.sax.TemplatesHandler;

import static java.lang.Math.PI;

public class Align {
    HardwareMap hwMap;
    private double approachingPower = 0.4;
    private double turnPower = 0.3;
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

    public void setPower(double approachingPower, double turnPower) {
        this.approachingPower = approachingPower;
        this.turnPower = turnPower;
    }

    public void foundation(FieldPosition f) {
        boolean correctRotation = false;

        while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
            if ((f == FieldPosition.RED_QUARY || f == FieldPosition.RED_FOUNDATION) && !correctRotation) {
                if (externalHeading >= 285 || externalHeading < 90) {
                    setRightPower(-turnPower);
                    setLeftPower(turnPower);
                } else if (externalHeading <= 275 && externalHeading >= 90) {
                    setRightPower(turnPower);
                    setLeftPower(-turnPower);
                }
                opMode.telemetry.addData("Target Heading", 270 + "°");
                opMode.telemetry.addData("Current Heading", externalHeading);
            } else if ((f == FieldPosition.BLUE_QUARY || f == FieldPosition.BLUE_FOUNDATION) && !correctRotation) {
                if (externalHeading <= 95 || externalHeading >= 270) {
                    setRightPower(turnPower);
                    setLeftPower(-turnPower);
                } else if (externalHeading >= 105 && externalHeading < 270) {
                    setRightPower(-turnPower);
                    setLeftPower(turnPower);
                }
                opMode.telemetry.addData("Target Heading", 90 + "°");
                opMode.telemetry.addData("Current Heading", externalHeading);
            }

            if ((externalHeading > 95 && externalHeading < 105) || (externalHeading < 285 && externalHeading > 275) &&
                    !correctRotation) {
                stop();
                correctRotation = true;
                opMode.telemetry.addData("Target Heading", "AT TARGET (±1°)");
                opMode.telemetry.addData("Current Heading", externalHeading);
            }

            if (correctRotation && hwMap.foundationDetectLeft.getState() && hwMap.foundationDetectRight.getState()) {
                setPower(-approachingPower);
                opMode.telemetry.addData("Target Heading", "AT TARGET (±3°)");
                opMode.telemetry.addData("Current Heading", externalHeading);
                opMode.telemetry.addData("LimitSwitchLeft", !hwMap.foundationDetectLeft.getState());
                opMode.telemetry.addData("LimitSwitchRight", !hwMap.foundationDetectRight.getState());
            } else if (correctRotation && !hwMap.foundationDetectLeft.getState() && !hwMap.foundationDetectRight.getState()) {
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                opMode.telemetry.addData("Target Heading", "AT TARGET (±3°)");
                opMode.telemetry.addData("Current Heading", externalHeading);
                opMode.telemetry.addData("LimitSwitchLeft", "Already Pressed");
                opMode.telemetry.addData("LimitSwitchRight", "Already Pressed");
                opMode.telemetry.update();
                stop();
                break;
            } else if (correctRotation && (!hwMap.foundationDetectLeft.getState() || !hwMap.foundationDetectRight.getState())) {
                if (!hwMap.foundationDetectLeft.getState()) {
                    hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                    opMode.telemetry.addData("Target Heading", "AT TARGET (±3°)");
                    opMode.telemetry.addData("Current Heading", externalHeading);
                    opMode.telemetry.addData("LimitSwitchLeft", "Already Pressed");
                    try {
                        Thread.sleep(300);
                    } catch (Exception e) {
                    }

                    while (hwMap.foundationDetectRight.getState()) {
                        setLeftPower(-turnPower);
                        setRightPower(turnPower);
                    }
                    opMode.telemetry.addData("LimitSwitchRight", "Already Pressed");
                    hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);
                    stop();
                    break;
                } else if (!hwMap.foundationDetectRight.getState()) {
                    hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);
                    opMode.telemetry.addData("Target Heading", "AT TARGET (±3°)");
                    opMode.telemetry.addData("Current Heading", externalHeading);
                    opMode.telemetry.addData("LimitSwitchRight", "Already Pressed");
                    try {
                        Thread.sleep(300);
                    } catch (Exception e) {
                    }

                    while (hwMap.foundationDetectLeft.getState()) {
                        setLeftPower(turnPower);
                        setRightPower(-turnPower);
                    }
                    hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                    opMode.telemetry.addData("LimitSwitchLeft", "Already Pressed");
                    stop();
                    break;
                }
                opMode.telemetry.update();
            }
            opMode.telemetry.update();
        }

    }

    public void skystoneRed(int position) {
        double distanceAway = -1;
        double theta = -1;
        double initLeft = getOdometry()[0];
        double initRight = getOdometry()[1];
        double initSide = getOdometry()[2];
        boolean isAligned = false;
        int imgWidth = -1;

        switch (position) {
            case 4:
                position = 2;
                break;
            case 5:
                position = 1;
                break;
            case 6:
                position = 0;
                break;
        }

        while (!isAligned && !opMode.isStopRequested()) {
            List<Recognition> detectedObj = this.detectedObj;
            // step through the list of recognitions and display boundary info.
            if (!opMode.isStopRequested() && detectedObj != null) {
                double[] center = new double[detectedObj.size()];
                ArrayList<Double> temp = new ArrayList<>();
                for (int i = 0; i < detectedObj.size(); i++) {
                    center[i] = detectedObj.get(i).getLeft() + detectedObj.get(i).getWidth() / 2;
                    temp.add((double) detectedObj.get(i).getLeft());
                    distanceAway = TFODCalc.getDistanceToObj(127,
                            detectedObj.get(i).getImageHeight(), detectedObj.get(i).getHeight());
                   // theta = TFODCalc.getAngleOfStone(detectedObj.get(i).getWidth(), distanceAway).get(0);
                    imgWidth = detectedObj.get(i).getImageWidth();
                }

                if(detectedObj.size() != 0) {
                    Collections.sort(temp);

                    int index = -1;
                    for (int i = 0; i < center.length; i++) {
                        if (center[i] == temp.get(position))
                            index = i;
                    }

                    if (center[index] >= imgWidth / 2d - 15 && center[index] <= imgWidth / 2d + 25) {
                        stop();
                        isAligned = true;
                    } else if (center[index] <= imgWidth / 2d - 15) {
                        setRightPower(turnPower);
                        setLeftPower(-turnPower);
                    } else if (center[index] >= imgWidth / 2d + 15) {
                        setRightPower(-turnPower);
                        setLeftPower(turnPower);
                    }
                }

                //if(distanceAway < 12 && theta > 35)
                //    setPower(-approachingPower);

                /*if (distanceAway < 15) {
                    stop();
                    isAligned = true;
                }*/
            }

           /* intake(TeleopConstants.intakePower);

            double left = getOdometry()[0];
            double right = getOdometry()[1];

            if (!collected && isAligned && !opMode.isStopRequested()) {
                double averageDelta = (getOdometry()[0] - left) + (getOdometry()[1] - right) / 2;
                double inchesMoved = averageDelta / 1400d * 2 * PI * 2;

                if (inchesMoved > 15 ) {
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
                    else if (deltaS < -250)
                        strafe(approachingPower, false);

                    if (deltaS <= 250 && deltaS >= -250) {
                        stop();
                        atOriginalPos = true;
                    }
                }
            }*/
        }
    }

    public void updateTFOD(List<Recognition> detectedObj) {
        this.detectedObj = detectedObj;
    }

    public void updateExternalHeading(double externalHeading) {
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

    private void stop() {
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

    private void strafe(double power, boolean right) {
        if (!right)
            power = -power;

        hwMap.frontRight.setPower(-power);
        hwMap.frontLeft.setPower(power);
        hwMap.backRight.setPower(power);
        hwMap.backLeft.setPower(-power);
    }
}
