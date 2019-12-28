package org.firstinspires.ftc.teamcode.Experimental.Movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Experimental.Units.Vector;
import org.firstinspires.ftc.teamcode.PID.util.LynxModuleUtil;

public class MecanumDrive {

    /* --==UPDATE ALL PRE-DEFINED CONSTANTS HERE==-- */

    private double encoderTicksPerInch = 455;
    private double wheelRadius = 5.0; //in inches
    private double gearRatio = 1 / 1;

    /* --------------------------------------------- */

    /*
           FIELD COORDINATES LAYOUT
                      X
                      /\ 72
                      |
                      |
                      |
                      |
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

    private BNO055IMU imu;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private Vector currentPos;
    private boolean renewedPos = false;
    private double turnCorrectorThreshold = 0.1;    //0 - 1: Allowed Error, 1 = OFF, 0 = 100% Accuracy Required
    private double driveCorrectorThreshold = 0.1;   //0 - 1: Allowed Error, 1 = OFF, 0 = 100% Accuracy Required

    public MecanumDrive(HardwareMap hwMap, BNO055IMU imu, DcMotorEx frontLeft, DcMotorEx frontRight,
                        DcMotorEx backLeft, DcMotorEx backRight, PIDFCoefficients coefficients){
        LynxModuleUtil.ensureMinimumFirmwareVersion(hwMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        this.imu = imu;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        setPID(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    public void runTrajectory(Vector estimatedPos, Trajectory trajectory){
        if(estimatedPos.getParamType() != 4)
           throw new IllegalArgumentException("Estimated Position must ONLY contain an X, Y, and Heading coordinate.");
        else
            this.currentPos = estimatedPos;

        renewedPos = true;
        for(Vector v : trajectory.getVectors()){
            if(v.getParamType() == 4){
                throw new IllegalArgumentException("All trajectory vectors must contain a power and MovementBehavior.");
            } else {
                switch (v.getParamType()) {
                    case 0:
                        break;
                    case 1:
                        break;
                    case 2:
                        break;
                    case 3:
                        break;
                }
            }
        }
    }

    private void turnTo(double theta, double turnPower){
        double imuData = imu.getAngularOrientation().firstAngle;
        boolean atTarget = false;

        if(theta >= 0){
            setPowerLeft(turnPower);
            setPowerRight(-turnPower);
        } else {
            setPowerLeft(-turnPower);
            setPowerRight(turnPower);
        }

        while(!atTarget) {
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
    }

    private void encoderDrive(double power, double inches){
        if(inches < 0){
            throw new IllegalArgumentException("Distance (in.) parameter for encoderDrive may not be negative.");
        } else {
            double initAvg = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backLeft.getCurrentPosition() +
                    backRight.getCurrentPosition()) / 4;
            boolean atTargetDistance = false;

            setPowerAll(power);

            while (!atTargetDistance) {
                double currAvg = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backLeft.getCurrentPosition() +
                        backRight.getCurrentPosition()) / 4;
                if (Math.abs(currAvg - initAvg) * encoderTicksPerInch >= inches)
                    atTargetDistance = true;
            }

            setPowerAll(0);
        }
    }

    private void lineTo(Vector endPos){
        if(endPos.getParamType() != 2){
            throw new IllegalArgumentException("A heading parameter may not be defined for LineTo move function.");
        } else {
            double deltaX = endPos.getX() - currentPos.getX();
            double deltaY = endPos.getY() - currentPos.getY();
            double theta = Math.atan(deltaY / deltaX);

            turnTo(theta, endPos.getPower());

            double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

            encoderDrive(endPos.getPower(), distance);
        }
    }

    private void setPID(DcMotor.RunMode runMode, PIDFCoefficients coefficients){
        frontLeft.setPIDFCoefficients(runMode, coefficients);
        frontRight.setPIDFCoefficients(runMode, coefficients);
        backLeft.setPIDFCoefficients(runMode, coefficients);
        backRight.setPIDFCoefficients(runMode, coefficients);
    }

    private void setPower(double frontL, double frontR, double backL, double backR){
        frontLeft.setPower(frontL);
        frontRight.setPower(frontR);
        backLeft.setPower(backL);
        backRight.setPower(backR);
    }

    private void setPowerAll(double pow){
        frontLeft.setPower(pow);
        frontRight.setPower(pow);
        backLeft.setPower(pow);
        backRight.setPower(pow);
    }

    private void setPowerRight(double pow){
        frontRight.setPower(pow);
        backRight.setPower(pow);
    }

    private void setPowerLeft(double pow){
        frontLeft.setPower(pow);
        backLeft.setPower(pow);
    }

    private void setMode(DcMotor.RunMode runMode){
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    private void updateEncoders(){
        Thread update = new Thread(){
            public void run(){
                while(!renewedPos) {
                    double fl = frontLeft.getCurrentPosition();
                    double fr = frontRight.getCurrentPosition();
                    double bl = backLeft.getCurrentPosition();
                    double br = backRight.getCurrentPosition();
                    double heading = imu.getAngularOrientation().firstAngle;

                    try {
                        Thread.sleep(50);
                    } catch (Exception e){}

                    double deltafl = frontLeft.getCurrentPosition() - fl;
                    double deltafr = frontRight.getCurrentPosition() - fr;
                    double deltabl = backLeft.getCurrentPosition() - bl;
                    double deltabr = backRight.getCurrentPosition() - br;
                    double deltaHeading = imu.getAngularOrientation().firstAngle - heading;

                    if(sameSigns(new double[] {deltafl, deltabl}) && sameSigns(new double[] {deltafr, deltabr})){
                        double arcLength = Math.max(avg(new double[] {deltabl, deltafl}), avg(new double[] {deltabr, deltafr})) +
                                Math.min(avg(new double[] {deltabl, deltafl}), avg(new double[] {deltabr, deltafr}));
                        //double arcAngle =
                        double currX = currentPos.getX();
                        double currY = currentPos.getY();
                    }
                }
            }
        };
    }

    private double encoderTicksToInches(double ticks){
        return ticks * gearRatio / encoderTicksPerInch * 2 * Math.PI * wheelRadius;
    }

    private double avg(double[] nums){
        double avg = 0;
        for(double i : nums)
            avg += i;
        return avg / nums.length;
    }

    private boolean sameSigns(double[] nums){
        if(nums.length > 1) {
            boolean neg = false;
            if(nums[0] < 0)
                neg = true;

            for (int i = 1; i < nums.length; i++)
                if(neg && nums[i] > 0 || !neg && nums[i] < 0)
                    return false;
        }
        return true;
    }

    private double rad(double rad) { return Math.toRadians(rad); }
    private double deg(double deg) { return Math.toDegrees(deg); }
}
