package org.firstinspires.ftc.teamcode.Experimental.Units;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.PID.util.LynxModuleUtil;

public class ConstraintsAndConstants {

    //////////////--==Mandatory Wheel Characteristics==--//////////////

    private MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(GoBILDA5202Series.class);

    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public BNO055IMU imu;
    public LinearOpMode currentOpMode;

    public PIDCoefficients coefficients = new PIDCoefficients(35, 0.5, 2.5);
    public AccelerationProperties accelerationProperties = new AccelerationProperties(0.25,0.2);
    //AccProportion: % of movement which the robot accelerates/decelerates (0 - 0.5)
    //AccRate: Accelerates by this much motor power per increment (0 - 1)

    public CorrectionThresholds correctionThresholds = new CorrectionThresholds(0.1,0.1);
    //0 - 1: Allowed Drive or Rotational Error, 1 = OFF, 0 = 100% Accuracy Required


    public double motorGearRatio = (99.5 / 13.7) * 1.0 / 1.0;
    public double wheelRadius = 2.0;
    public double motorEncoderTicksPerRev = 383.6;

    //////////////--==Odometry ONLY==--//////////////

    public DcMotorEx xLeftOdo;
    public DcMotorEx xRightOdo;
    public DcMotorEx yMiddleOdo;

    public double odometryEncoderTicksPerRev = 1560.0;
    public double odometryGearRatio = 1.0 / 1.0;
    public double odometryWheelRadius = 1.25;

    ////////////////////////////////////////////////

    public ConstraintsAndConstants(HardwareMap hwMap, LinearOpMode opMode) {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hwMap);

        backLeft = (DcMotorEx) hwMap.get(DcMotor.class, "backLeft");
        backRight = (DcMotorEx) hwMap.get(DcMotor.class, "backRight");
        frontLeft = (DcMotorEx) hwMap.get(DcMotor.class, "frontLeft");
        frontRight = (DcMotorEx) hwMap.get(DcMotor.class, "frontRight");
        imu = hwMap.get(BNO055IMU.class, "imu");
        xLeftOdo = (DcMotorEx) hwMap.get(DcMotor.class, "leftIntake");
        xRightOdo = (DcMotorEx) hwMap.get(DcMotor.class, "liftTwo");;
        yMiddleOdo = (DcMotorEx) hwMap.get(DcMotor.class, "rightIntake");;

        currentOpMode = opMode;

        opMode.telemetry.addData("ROBOT STATUS", "Calibrating IMU...");
        opMode.telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        opMode.telemetry.addData("ROBOT STATUS", "IMU Successfully Calibrated");
        opMode.telemetry.update();
    }

    public boolean odometryLocalized(){
        return xLeftOdo != null && xRightOdo != null && yMiddleOdo != null;
    }

    public double wheelEncoderTicksToInches(double ticks) {
        return wheelRadius * 2 * Math.PI * motorGearRatio * ticks / MOTOR_CONFIG.getTicksPerRev(); //2786 ticks per rev for Gobilda5202
    }

    public double wheelInchesToEncoderTicks(double inches){
        return (inches * MOTOR_CONFIG.getTicksPerRev()) / (wheelRadius * 2 * Math.PI * motorGearRatio);
    }

    public double odometryEncoderTicksToInches(double ticks){
        return ticks / odometryEncoderTicksPerRev * 2 * Math.PI * odometryWheelRadius * odometryGearRatio;
    }

    public double odometryInchesToEncoderTicks(double inches){
        return inches / (2 * Math.PI * odometryWheelRadius * odometryGearRatio) * odometryEncoderTicksPerRev;
    }

    public double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        return (435.0 * MOTOR_CONFIG.getTicksPerRev() / 60.0);
    }

    public double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / getTicksPerSec();
    }
}
