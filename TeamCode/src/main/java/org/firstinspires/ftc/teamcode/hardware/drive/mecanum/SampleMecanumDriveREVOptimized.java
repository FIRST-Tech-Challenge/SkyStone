package org.firstinspires.ftc.teamcode.hardware.drive.mecanum;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.hardware.drive.localizer.TemporaryLocalizer;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants.encoderTicksToInches;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
@Config
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private ExpansionHubEx hub;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    //leftFront Port 0
    //leftRear Port 1
    //rightFront Port 2
    //rightRear Port 3

    public static double angleCorrection = 0.05;
    private double startingAngle = 0;

    //If the boolean below is false, then it will attempt to store a new angle for correction. If it is true, then the robot is translating and is referencing the previous angle.
    private boolean ifStartingAngle = false;

    private static SampleMecanumDriveREVOptimized drive;

    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;


    public static SampleMecanumDriveREVOptimized getInstance(HardwareMap hardwareMap){
        if(drive == null){
            drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        }
        return drive;
    }

    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "leftRear");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "rightRear");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        setLocalizer(new TemporaryLocalizer(hardwareMap, imu));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }


    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        rightFront.setPower(v3);
        rightRear.setPower(v2);
        leftRear.setPower(v1);
    }

//    @Override
//    public void setDrivePower(Pose2d pose){
//
//        Double velocityX = pose.getX();
//        Double velocityY = pose.getY();
//        Double velocityR = pose.getHeading();
//
//        double anglePowerCorrection = 0;
//        if(velocityR < 0.05 && velocityR > -0.05){
//            if(!ifStartingAngle){
//                ifStartingAngle = true;
//                startingAngle = drive.getPoseEstimate().getHeading();
//            }
//            anglePowerCorrection = angleCorrection * (startingAngle - drive.getPoseEstimate().getHeading());
//        } else if((velocityR >= 0.05 || velocityR <= -0.05) || ((velocityX <= 0.05 && velocityX >= -0.05) && (velocityY <= 0.05 && velocityY >= -0.05))){
//            ifStartingAngle = false;
//        }
//
//        List<Double> translationValues = new ArrayList<>();
//        //Front Left
//        translationValues.add(velocityX + velocityY);
//
//        //Front Right
//        translationValues.add(velocityX - velocityY);
//
//        //Back Left
//        translationValues.add(velocityX - velocityY);
//
//        //Back Right
//        translationValues.add(velocityX + velocityY);
//
//        List<Double> rotationValues = new ArrayList<>();
//        //Front Left
//        rotationValues.add(velocityR + anglePowerCorrection);
//
//        //Front Right
//        rotationValues.add(-velocityR - anglePowerCorrection);
//
//        //Back Left
//        rotationValues.add(velocityR + anglePowerCorrection);
//
//        //Back Right
//        rotationValues.add(-velocityR - anglePowerCorrection);
//
//        double scaleFactor = 1;
//        double tmpScale = 1;
//
//        for (int i = 0; i < 4; i++) {
//            if (Math.abs(translationValues.get(i) + rotationValues.get(i)) > 1) {
//                tmpScale = (1 - rotationValues.get(i)) / translationValues.get(i);
//            } else if (translationValues.get(i) + rotationValues.get(i) < -1) {
//                tmpScale = (rotationValues.get(i) - 1) / translationValues.get(i);
//            }
//            if (tmpScale < scaleFactor) {
//                scaleFactor = tmpScale;
//            }
//        }
//
//
//        List<Double> valuesScaled = new ArrayList<>();
//        for (int i = 0; i < 4; i++) {
//            valuesScaled.add(translationValues.get(i) * scaleFactor + rotationValues.get(i));
//        }
//
//        setMotorPowers(valuesScaled.get(0),valuesScaled.get(2),valuesScaled.get(3),valuesScaled.get(1));
//
//    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}