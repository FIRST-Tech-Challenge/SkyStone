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
    private boolean failsafe = false;
    private double initPos = 0;

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
        boolean correctRotation = false;    //TODO Add failsafe

        while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
            double externalHeading = this.externalHeading;
            if ((f == FieldPosition.RED_QUARY || f == FieldPosition.RED_FOUNDATION) && !correctRotation) {
                if (externalHeading >= 270 || externalHeading < 90) {
                    setRightPower(-turnPower);
                    setLeftPower(turnPower);
                } else if (externalHeading <= 269 && externalHeading >= 90) {
                    setRightPower(turnPower);
                    setLeftPower(-turnPower);
                }
                opMode.telemetry.addData("Target Heading", 270 + "°");
                opMode.telemetry.addData("Current Heading", externalHeading);
            } else if ((f == FieldPosition.BLUE_QUARY || f == FieldPosition.BLUE_FOUNDATION) && !correctRotation) {
                if (externalHeading <= 94 || externalHeading >= 270) {
                    setRightPower(turnPower);
                    setLeftPower(-turnPower);
                } else if (externalHeading >= 95 && externalHeading < 270) {
                    setRightPower(-turnPower);
                    setLeftPower(turnPower);
                }
                opMode.telemetry.addData("Target Heading", 90 + "°");
                opMode.telemetry.addData("Current Heading", externalHeading);
            }

            if (((externalHeading > 94 && externalHeading < 95) || (externalHeading < 270 && externalHeading > 269)) &&
                    !correctRotation) {
                stop();
                correctRotation = true;
                opMode.telemetry.addData("Target Heading", "AT TARGET (±1°)");
                opMode.telemetry.addData("Current Heading", externalHeading);
                initPos = System.currentTimeMillis();
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

            if(correctRotation && Math.abs(System.currentTimeMillis() - initPos) > 6000){
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
                stop();
                break;
            }
            opMode.telemetry.update();
        }

    }

    public void skystoneRed(int position) {
        double distanceAway = -1;
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
            if (!opMode.isStopRequested() && detectedObj != null) {
                ArrayList<Double> temp = new ArrayList<>();
                for (int i = 0; i < detectedObj.size(); i++) {
                    temp.add((double) detectedObj.get(i).getLeft() + detectedObj.get(i).getWidth() / 2);
                    distanceAway = TFODCalc.getDistanceToObj(127,
                            detectedObj.get(i).getImageHeight(), detectedObj.get(i).getHeight());
                    imgWidth = detectedObj.get(i).getImageWidth();
                }

                if(!temp.isEmpty()) {
                    Collections.sort(temp);

                    if(position >= temp.size())
                        position = temp.size() - 1;

                    opMode.telemetry.addData("Current Position", temp.get(position));
                    opMode.telemetry.addData("Target Bounds", (imgWidth / 2d - 15) + " - " + (imgWidth / 2d + 15));
                    opMode.telemetry.addData("Raw, Sorted Data", temp);

                    if (temp.get(position) >= imgWidth / 2d - 15 && temp.get(position) <= imgWidth / 2d + 15) {
                        stop();
                        isAligned = true;
                        opMode.telemetry.addData("Executed Command", "STOP");
                    } else if (temp.get(position) <= imgWidth / 2d - 15) {
                        setRightPower(turnPower);
                        setLeftPower(-approachingPower);
                        opMode.telemetry.addData("Executed Command", "RIGHT");
                    } else if (temp.get(position) >= imgWidth / 2d + 15) {
                        setRightPower(-approachingPower);
                        setLeftPower(turnPower);
                        opMode.telemetry.addData("Executed Command", "LEFT");
                    }
                    opMode.telemetry.update();
                }
            }
        }
    }

    private ArrayList<Double> iterate(){
        ArrayList<Double> temp = new ArrayList<>();
        for (int i = 0; i < detectedObj.size(); i++) {
            temp.add((double) detectedObj.get(i).getLeft() + detectedObj.get(i).getWidth() / 2);
            //distanceAway = TFODCalc.getDistanceToObj(127,
            //        detectedObj.get(i).getImageHeight(), detectedObj.get(i).getHeight());
        }
        return temp;
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
