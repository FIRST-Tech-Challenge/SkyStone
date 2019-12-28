package org.firstinspires.ftc.teamcode.Experimental.Movement;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.Experimental.Units.ConstraintsAndConstants;
import org.firstinspires.ftc.teamcode.Experimental.Units.Vector;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

public class OdometryDrive {
    /*
           FIELD COORDINATES LAYOUT
                      X
                      /\ 72
                      |      _ _
                      |     |   |
                      |     | R | <-- Robot at ~(48, 48)
                      |      - -
                      |
                      |
        < - - - - - - - - - - - - - > Y
       -72            |            72
                      |
                      |
                      |
                      |
                      |
                      \/ -72
     */

    private Vector currentPos = new Vector(0, 0, 0);
    private boolean renewedPos = false;
    private ConstraintsAndConstants constants;
    private ArrayList<String> debug = new ArrayList<>();

    public OdometryDrive(HardwareMap hwMap, LinearOpMode opMode) {
        constants = new ConstraintsAndConstants(hwMap, opMode);

        if (!constants.odometryLocalized())
            throw new NullPointerException("Odometry wheels have not been initialized! Odometry wheels are REQUIRED for\n" +
                    " motionplanning through OdometryDrive.");

        setPID(DcMotor.RunMode.RUN_USING_ENCODER, constants.coefficients);
    }

    public void runTrajectory(Trajectory trajectory) {
        Vector estimatedPos = trajectory.getStartingVector();
        if (estimatedPos == null)
            throw new NullPointerException("Initial Estimated Vector is null.");
        else {
            if (estimatedPos.getParamType() != 4)
                throw new IllegalArgumentException("Estimated Position must ONLY contain an X, Y, and Heading coordinate.");
            else
                this.currentPos = new Vector(estimatedPos.getX(), estimatedPos.getY(), estimatedPos.getHeading());
        }
        renewedPos = false;

        try {
            Thread.sleep(20);
        } catch (Exception e) {
        }

        updateEncoders();

        for (Vector v : trajectory.getVectors()) {
            if (v.getParamType() == 4) {
                throw new IllegalArgumentException("All trajectory vectors must contain a power and MovementBehavior.");
            } else {
                switch (v.getParamType()) {
                    case 0:
                        break;
                    case 1:
                        v.getAction().run();
                        break;
                    case 2:
                        if (v.getMoveBehavior() == Vector.MoveBehavior.LineTo)
                            lineTo(v);
                        else if (v.getMoveBehavior() == Vector.MoveBehavior.StrafeTo)
                            strafeTo(v);
                        else
                            throw new IllegalArgumentException("MovementBehavior invalid for defined constructor.");
                        break;
                    case 3:
                        if (v.getMoveBehavior() == Vector.MoveBehavior.RotateTo)
                            turnTo(v.getHeading(), v.getPower());
                        else
                            throw new IllegalArgumentException("MovementBehavior invalid for defined constructor.");
                        break;
                }
            }
        }
        renewedPos = true;

        try {
            Thread.sleep(20);
        } catch (Exception e) {
        }

        currentPos = null;
    }

    private void lineTo(Vector endPos) {
        if (endPos.getParamType() != 2) {
            throw new IllegalArgumentException("A heading parameter may not be defined for LineTo move function.");
        } else {
            double deltaX = endPos.getX() - currentPos.getX();
            double deltaY = endPos.getY() - currentPos.getY();
            if (deltaX != 0 && deltaY != 0) {
                double theta;
                try {
                    theta = Math.atan(deltaY / deltaX);
                } catch (Exception e) {
                    if (deltaY > 0)
                        theta = Math.PI / 2;
                    else
                        theta = (3 * Math.PI) / 2;
                }

                if (deltaX < 0 && deltaY < 0 || deltaY > 0 && deltaX < 0)
                    theta += Math.PI;
                else if (deltaY < 0 && deltaX > 0)
                    theta += 2 * Math.PI;

                turnTo(theta, endPos.getPower());
                currentPos.setHeading(theta);

                double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

                debug.add(System.currentTimeMillis() + ": \"LineTo\" -- DeltaX & DeltaY = (" + deltaX + " // " +
                        deltaY + ") // Theta = " + theta + " // Distance = " + distance + "\n");

                encoderDrive(endPos.getPower(), distance);

                correctTo(endPos);
            }
        }
    }

    private void strafeTo(Vector endPos) {
        if (endPos.getParamType() != 2) {
            throw new IllegalArgumentException("A heading parameter may not be defined for StrafeTo move function.");
        } else {
            double deltaX = endPos.getX() - currentPos.getX();
            double deltaY = endPos.getY() - currentPos.getY();
            if (deltaX != 0 && deltaY != 0) {
                double theta;
                try {
                    theta = Math.atan(deltaY / deltaX);
                } catch (Exception e) {
                    if (deltaY > 0)
                        theta = Math.PI / 2;
                    else
                        theta = (3 * Math.PI) / 2;
                }
                double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

                if (deltaX < 0 && deltaY < 0 || deltaY > 0 && deltaX < 0)
                    theta += Math.PI;
                else if (deltaY < 0 && deltaX > 0)
                    theta += 2 * Math.PI;

                if (currentPos.getHeading() <= Math.PI && theta > Math.PI) {
                    theta -= Math.PI;
                    turnTo(theta - Math.PI / 2, endPos.getPower());
                    debug.add(System.currentTimeMillis() + ": \"StrafeTo\" -- DeltaX & DeltaY = (" + deltaX + " " +
                            deltaY + ") // Theta = " + (theta - Math.PI / 2) + " // Distance = " + distance + "\n");
                    encoderStrafe(-endPos.getPower(), distance);

                    endPos.setHeading(theta - Math.PI / 2);
                    correctTo(endPos);
                } else if (currentPos.getHeading() > Math.PI && theta <= Math.PI) {
                    theta += Math.PI;
                    turnTo(theta + Math.PI / 2, endPos.getPower());
                    debug.add(System.currentTimeMillis() + ": \"StrafeTo\" -- DeltaX & DeltaY = (" + deltaX + " " +
                            deltaY + ") // Theta = " + (theta + Math.PI / 2) + " // Distance = " + distance + "\n");
                    encoderStrafe(endPos.getPower(), distance);

                    endPos.setHeading(theta + Math.PI / 2);
                    correctTo(endPos);
                }
            }
        }
    }

    private void turnTo(double theta, double turnPower) {
        double imuData = constants.imu.getAngularOrientation().firstAngle;
        boolean atTarget = false;

        if (theta > 0) {
            setPowerLeft(turnPower);
            setPowerRight(-turnPower);
        } else if (theta < 0) {
            setPowerLeft(-turnPower);
            setPowerRight(turnPower);
        } else {
            atTarget = true;
        }

        while (!atTarget) {
            if (theta + rad(0.5) > 2 * Math.PI) {
                if (imuData > theta - rad(0.5) || imuData < theta + rad(0.5) - Math.PI * 2)
                    atTarget = true;
            } else if (theta - rad(0.5) < 0) {
                if (imuData < theta + rad(0.5) || imuData > theta - rad(0.5) + Math.PI * 2)
                    atTarget = true;
            } else {
                if (imuData > theta - rad(0.5) && imuData < theta + rad(0.5))
                    atTarget = true;
            }
        }
        setPowerAll(0);

        debug.add(System.currentTimeMillis() + ": \"RotateTo\" -- IMU = " + imuData + " // Theta = " + theta + "\n");

        correctTo(new Vector(currentPos.getX(), currentPos.getY(), theta));
    }

    private void encoderDrive(double power, double inches) {
        if (power == 0)
            throw new IllegalArgumentException("Defined power for encoderDrive cannot = 0.");
        if (inches < 0) {
            throw new IllegalArgumentException("Distance (in.) parameter for encoderDrive may not be negative.");
        } else {
            double initAvg = (constants.frontLeft.getCurrentPosition() + constants.frontRight.getCurrentPosition() + constants.backLeft.getCurrentPosition() +
                    constants.backRight.getCurrentPosition()) / 4;

            constants.accelerationProperties.accelerate(constants, initAvg, power, inches);
            debug.add(constants.accelerationProperties.getDebug());

            setPowerAll(power);

            constants.accelerationProperties.decelerate(constants, initAvg, power, inches);
            debug.add(constants.accelerationProperties.getDebug());

            setPowerAll(0);
        }
    }

    private void encoderStrafe(double power, double inches) {
        double topLbotR = avg(new double[]{constants.frontLeft.getCurrentPosition(), constants.backRight.getCurrentPosition()});
        double topRbotL = avg(new double[]{constants.frontRight.getCurrentPosition(), constants.backLeft.getCurrentPosition()});
        double counts = constants.motorEncoderTicksPerRev * (inches / 2 * Math.PI * constants.wheelRadius) *
                constants.motorGearRatio;

        debug.add(System.currentTimeMillis() + ": \"EncoderStrafe\" -- TargetCounts = " + counts + "\n");

        if (inches > 0) {
            constants.accelerationProperties.accelerate(constants, getCurrentPositionAll(),
                    new double[]{power, -power, -power, power}, new double[]{constants.wheelInchesToEncoderTicks(counts),
                            constants.wheelInchesToEncoderTicks(counts), constants.wheelInchesToEncoderTicks(counts),
                            constants.wheelInchesToEncoderTicks(counts)});
            debug.add(constants.accelerationProperties.getDebug());
            //while(avg(new double[] {constants.frontLeft.getCurrentPosition(), constants.backRight.getCurrentPosition()}) - topLbotR < counts){
            constants.frontLeft.setPower(power);
            constants.backRight.setPower(power);
            constants.frontRight.setPower(-power);
            constants.backLeft.setPower(-power);

            constants.accelerationProperties.decelerate(constants, getCurrentPositionAll(),
                    new double[]{power, -power, -power, power}, new double[]{constants.wheelInchesToEncoderTicks(counts),
                            constants.wheelInchesToEncoderTicks(counts), constants.wheelInchesToEncoderTicks(counts),
                            constants.wheelInchesToEncoderTicks(counts)});
            debug.add(constants.accelerationProperties.getDebug());
            //}
        } else {
            //while(avg(new double[] {constants.frontLeft.getCurrentPosition(), constants.backRight.getCurrentPosition()}) - topLbotR > -counts){
            constants.accelerationProperties.accelerate(constants, getCurrentPositionAll(),
                    new double[]{-power, power, power, -power}, new double[]{constants.wheelInchesToEncoderTicks(counts),
                            constants.wheelInchesToEncoderTicks(counts), constants.wheelInchesToEncoderTicks(counts),
                            constants.wheelInchesToEncoderTicks(counts)});
            debug.add(constants.accelerationProperties.getDebug());

            constants.frontLeft.setPower(-power);
            constants.backRight.setPower(-power);
            constants.frontRight.setPower(power);
            constants.backLeft.setPower(power);

            constants.accelerationProperties.decelerate(constants, getCurrentPositionAll(),
                    new double[]{-power, power, power, -power}, new double[]{constants.wheelInchesToEncoderTicks(counts),
                            constants.wheelInchesToEncoderTicks(counts), constants.wheelInchesToEncoderTicks(counts),
                            constants.wheelInchesToEncoderTicks(counts)});
            debug.add(constants.accelerationProperties.getDebug());
            //}
        }

        setPowerAll(0);
    }

    private void setPID(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        PIDFCoefficients PIDF = new PIDFCoefficients(coefficients.p, coefficients.i, coefficients.d, constants.getMotorVelocityF());
        constants.frontLeft.setPIDFCoefficients(runMode, PIDF);
        constants.frontRight.setPIDFCoefficients(runMode, PIDF);
        constants.backLeft.setPIDFCoefficients(runMode, PIDF);
        constants.backRight.setPIDFCoefficients(runMode, PIDF);
    }

    private void setPower(double frontL, double frontR, double backL, double backR) {
        constants.frontLeft.setPower(frontL);
        constants.frontRight.setPower(frontR);
        constants.backLeft.setPower(backL);
        constants.backRight.setPower(backR);
    }

    private void setPowerAll(double pow) {
        constants.frontLeft.setPower(pow);
        constants.frontRight.setPower(pow);
        constants.backLeft.setPower(pow);
        constants.backRight.setPower(pow);
    }

    private void setPowerRight(double pow) {
        constants.frontRight.setPower(pow);
        constants.backRight.setPower(pow);
    }

    private void setPowerLeft(double pow) {
        constants.frontLeft.setPower(pow);
        constants.backLeft.setPower(pow);
    }

    private void setMode(DcMotor.RunMode runMode) {
        constants.frontLeft.setMode(runMode);
        constants.frontRight.setMode(runMode);
        constants.backLeft.setMode(runMode);
        constants.backRight.setMode(runMode);
    }

    private void updateEncoders() {
        Thread update = new Thread() {
            public void run() {
                while (!renewedPos && constants.currentOpMode.opModeIsActive()) {
                    double rx = constants.xRightOdo.getCurrentPosition();
                    double lx = constants.xLeftOdo.getCurrentPosition();
                    double my = constants.yMiddleOdo.getCurrentPosition();
                    double theta = constants.imu.getAngularOrientation().firstAngle;

                    try {
                        Thread.sleep(50);
                    } catch (Exception e) {
                    }

                    double deltarx = constants.xRightOdo.getCurrentPosition() - rx;
                    double deltalx = constants.xLeftOdo.getCurrentPosition() - lx;
                    double deltamy = constants.yMiddleOdo.getCurrentPosition() - my;
                    double deltaTheta = constants.imu.getAngularOrientation().firstAngle;

                    if (deltaTheta - theta > Math.PI)
                        deltaTheta = deltaTheta - (theta + Math.PI * 2);
                    else if (deltaTheta - theta < Math.PI)
                        deltaTheta = theta - (deltaTheta + Math.PI * 2);
                    else
                        deltaTheta -= theta;

                    currentPos.setX(currentPos.getX() + constants.odometryEncoderTicksToInches(avg(new double[]{deltarx, deltalx})));
                    currentPos.setY(currentPos.getY() + constants.odometryEncoderTicksToInches(deltamy));
                    currentPos.setHeading(currentPos.getHeading() + deltaTheta);
                }
            }
        };
        update.start();
    }

    private void correctTo(Vector targetPos) {
        double dt = constants.correctionThresholds.getDriveCorrectionAllowedError();
        double rt = constants.correctionThresholds.getRotationCorrectionAllowedError();
        double strafePow = 0.65;
        double drivePow = 0.5;
        boolean xyAligned = false;

        while (!isBetween(targetPos.getX(), currentPos.getX() + dt, currentPos.getX() - dt) ||
                !isBetween(targetPos.getY(), currentPos.getY() + dt, currentPos.getY() - dt) ||
                !isBetween(targetPos.getHeading(), currentPos.getHeading() + rt,
                        currentPos.getHeading() - rt)) {

            if(!xyAligned)
                turnTo(0, drivePow);

            if (targetPos.getX() >= currentPos.getX() + dt || targetPos.getX() <= currentPos.getX() - dt && !xyAligned) {
                if (targetPos.getX() >= currentPos.getX() + dt) {
                    constants.frontLeft.setPower(-strafePow);
                    constants.backRight.setPower(-strafePow);
                    constants.frontRight.setPower(strafePow);
                    constants.backLeft.setPower(strafePow);
                } else if (targetPos.getX() <= currentPos.getX() - dt) {
                    constants.frontLeft.setPower(strafePow);
                    constants.backRight.setPower(strafePow);
                    constants.frontRight.setPower(-strafePow);
                    constants.backLeft.setPower(-strafePow);
                }
            } else if (targetPos.getY() >= currentPos.getY() + dt || targetPos.getY() <= currentPos.getY() - dt && !xyAligned) {
                if (targetPos.getY() >= currentPos.getY() + dt) {
                    constants.frontLeft.setPower(-drivePow);
                    constants.backRight.setPower(-drivePow);
                    constants.frontRight.setPower(-drivePow);
                    constants.backLeft.setPower(-drivePow);
                } else if (targetPos.getY() <= currentPos.getY() - dt) {
                    constants.frontLeft.setPower(drivePow);
                    constants.backRight.setPower(drivePow);
                    constants.frontRight.setPower(drivePow);
                    constants.backLeft.setPower(drivePow);
                }
            } else if(targetPos.getX() <= currentPos.getX() + dt && targetPos.getX() >= currentPos.getX() - dt &&
                    targetPos.getY() <= currentPos.getY() + dt && targetPos.getY() >= currentPos.getY() - dt)
                xyAligned = true;

            if (currentPos.getHeading() + rt > Math.PI * 2 && xyAligned) {
                double temp = (currentPos.getHeading() - rt) - (currentPos.getHeading() + rt - 2 * Math.PI) / 2;
                if (targetPos.getHeading() >= currentPos.getHeading() + rt - 2 * Math.PI && targetPos.getHeading() <
                        currentPos.getHeading() + rt - 2 * Math.PI + temp) {
                    constants.frontLeft.setPower(drivePow);
                    constants.backRight.setPower(-drivePow);
                    constants.frontRight.setPower(-drivePow);
                    constants.backLeft.setPower(drivePow);
                } else if (targetPos.getHeading() >= currentPos.getHeading() - rt) {
                    constants.frontLeft.setPower(-drivePow);
                    constants.backRight.setPower(drivePow);
                    constants.frontRight.setPower(drivePow);
                    constants.backLeft.setPower(-drivePow);
                }
            } else if (currentPos.getHeading() - rt < Math.PI * 2 && xyAligned) {
                if (targetPos.getHeading() >= currentPos.getHeading() + rt) {
                    constants.frontLeft.setPower(drivePow);
                    constants.backRight.setPower(-drivePow);
                    constants.frontRight.setPower(-drivePow);
                    constants.backLeft.setPower(drivePow);
                } else if (targetPos.getHeading() <= currentPos.getHeading() - rt + Math.PI * 2) {
                    constants.frontLeft.setPower(-drivePow);
                    constants.backRight.setPower(drivePow);
                    constants.frontRight.setPower(drivePow);
                    constants.backLeft.setPower(-drivePow);
                }
            } else if (currentPos.getHeading() + rt <= Math.PI * 2 && currentPos.getHeading() - rt >= Math.PI * 2 && xyAligned) {
                if (targetPos.getHeading() >= currentPos.getHeading() + rt) {
                    constants.frontLeft.setPower(drivePow);
                    constants.backRight.setPower(-drivePow);
                    constants.frontRight.setPower(-drivePow);
                    constants.backLeft.setPower(drivePow);
                } else if (targetPos.getHeading() <= currentPos.getHeading() - rt) {
                    constants.frontLeft.setPower(-drivePow);
                    constants.backRight.setPower(drivePow);
                    constants.frontRight.setPower(drivePow);
                    constants.backLeft.setPower(-drivePow);
                }
            }
        }
        setPowerAll(0);
    }

    private double avg(double[] nums) {
        double avg = 0;
        for (double i : nums)
            avg += i;
        return avg / nums.length;
    }

    private boolean sameSigns(double[] nums) {
        if (nums.length > 1) {
            boolean neg = false;
            if (nums[0] < 0)
                neg = true;

            for (int i = 1; i < nums.length; i++)
                if (neg && nums[i] > 0 || !neg && nums[i] < 0)
                    return false;
        }
        return true;
    }

    private double[] getCurrentPositionAll() {
        return new double[]{constants.frontLeft.getCurrentPosition(), constants.frontRight.getCurrentPosition(),
                constants.backLeft.getCurrentPosition(), constants.backRight.getCurrentPosition()};
    }

    private double rad(double rad) {
        return Math.toRadians(rad);
    }

    private double deg(double deg) {
        return Math.toDegrees(deg);
    }

    private boolean isBetween(double target, double maxBounds1, double minBounds2) {
        return target <= maxBounds1 && target >= minBounds2;
    }

    public Vector getCurrentPosition() {
        return currentPos;
    }

    public double getIMUHeading() {
        return constants.imu != null ? constants.imu.getAngularOrientation().firstAngle : 0.0;
    }

    public String getDebug(){
        String log = debug.toString();
        log = log.replaceAll("]", "");
        log = log.replaceAll("\\[", "");
        log = log.replaceAll(",", "");
        return log;
    }

    public void saveLog(){
        String log = debug.toString();
        log = log.replaceAll("]", "");
        log = log.replaceAll("\\[", "");
        log = log.replaceAll(",", "");
        while(!constants.currentOpMode.isStopRequested()) {
            if(constants.currentOpMode.gamepad1.a) {
                writeFile(AppUtil.ROOT_FOLDER + "/CustomMotionPlanning/OdometryDrive_" +
                        System.currentTimeMillis() + ".txt", log);
                break;
            } else if(constants.currentOpMode.gamepad1.b)
                break;

            constants.currentOpMode.telemetry.addData("OdometryDrive","Press A to save logs. Press B " +
                    "or stop OpMode to cancel.");
            constants.currentOpMode.telemetry.update();
        }
    }

    private void writeFile(String filePath, String data) {
        try {
            File folder = new File(filePath.substring(0, filePath.lastIndexOf("/") + 1));
            folder.mkdirs();
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(new File(filePath), true));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }
}
