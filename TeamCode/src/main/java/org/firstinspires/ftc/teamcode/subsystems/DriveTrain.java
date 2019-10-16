package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.TestableGyro;
import org.westtorrancerobotics.lib.Location;
import org.westtorrancerobotics.lib.MecanumController;
import org.westtorrancerobotics.lib.MecanumDrive;

public class DriveTrain {

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private ModernRoboticsI2cGyro gyro;
    private MecanumController mecanumController;

    private ColorSensor lineSpotter;
    private static final int RED_THRESHOLD  = 5;
    private static final int BLUE_THRESHOLD = 5;

    private static DriveTrain instance = null;

    public static synchronized DriveTrain getInstance() {
        return instance != null ? instance : (instance = new DriveTrain());
    }

    private DriveTrain() {}

    public void init(HardwareMap hardwareMap) {
        leftFront  = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack  = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        lineSpotter = hardwareMap.get(ColorSensor.class, "line_color");

        MecanumDrive wheels = new MecanumDriveImpl(leftFront, leftBack, rightFront, rightBack, TestableGyro.generate(gyro));
        mecanumController = new MecanumController(wheels);
    }

    public void setMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        leftFront.setZeroPowerBehavior(mode);
        leftBack.setZeroPowerBehavior(mode);
        rightFront.setZeroPowerBehavior(mode);
        rightBack.setZeroPowerBehavior(mode);
    }

    public void spinDrive(double x, double y, double turn) {
        mecanumController.spinDrive(x, y, turn, MecanumDrive.TranslTurnMethod.EQUAL_SPEED_RATIOS);
    }

    public void calibrateGyro() {
        gyro.calibrate();
    }

    public boolean isCalibratingGyro() {
        return gyro.isCalibrating();
    }

    public void updateLocation() {
        mecanumController.updateLocation();
    }

    public void setLocationZero() {
        mecanumController.zeroDeadReckoner();
    }

    public void setLocation(Location l) {
        mecanumController.setLocation(l);
    }

    public Location getLocation() {
        return mecanumController.getLocation();
    }

    public boolean onRedLine() {
        return lineSpotter.red() > RED_THRESHOLD;
    }

    public boolean onBlueLine() {
        return lineSpotter.blue() > BLUE_THRESHOLD;
    }
}
