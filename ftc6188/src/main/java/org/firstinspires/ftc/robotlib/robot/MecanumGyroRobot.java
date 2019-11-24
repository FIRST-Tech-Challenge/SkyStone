package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumGyroRobot extends MecanumRobot
{
    public BNO055IMUImpl imu;

    public MecanumGyroRobot(HardwareMap hwMap, Telemetry telemetry, boolean teleOpMode)
    {
        super(hwMap, telemetry, teleOpMode);

        imu = hwMap.get(BNO055IMUImpl.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }

    @Override
    public void informationUpdate()
    {
        telemetry.addData("> IMU Output", "-----");
        telemetry.addData("Acceleration", imu.getAcceleration());
        telemetry.addData("Angle", imu.getAngularOrientation());
        telemetry.addData("Omega", imu.getAngularVelocity());

        telemetry.addData("> Wheel Positions", "-----");
        telemetry.addData("WheelPos FL", drivetrain.motorList[0].getCurrentPosition());
        telemetry.addData("WheelPos FR", drivetrain.motorList[1].getCurrentPosition());
        telemetry.addData("WheelPos RL", drivetrain.motorList[2].getCurrentPosition());
        telemetry.addData("WheelPos RR", drivetrain.motorList[3].getCurrentPosition());
        telemetry.addData("Current Pos Percent", drivetrain.getCurrentPosition());
        telemetry.addData("Current Pos", drivetrain.getCurrentPosition() * drivetrain.getTargetPosition());

        telemetry.addData("> Wheel Powers", "-----");
        telemetry.addData("WheelPower FL", drivetrain.motorList[0].getPower());
        telemetry.addData("WheelPower FR", drivetrain.motorList[1].getPower());
        telemetry.addData("WheelPower RL", drivetrain.motorList[2].getPower());
        telemetry.addData("WheelPower RR", drivetrain.motorList[3].getPower());

        telemetry.addData("> Drivetrain Info", "-----");
        telemetry.addData("Course Radians", drivetrain.getCourse());
        telemetry.addData("Course Degrees", drivetrain.getCourse() * Math.PI/180);
        telemetry.addData("Rotation Target", drivetrain.getRotation());
        telemetry.addData("Velocity Target", drivetrain.getVelocity());
        telemetry.addData("Current Pos", drivetrain.getCurrentPosition());
        telemetry.addData("Is Pos", drivetrain.isPositioning());

        telemetry.addData("> Servo Info", "-----");
        telemetry.addData("Servo Pos", "One: " + platformServos.getServoOne().getPosition() + " Two: " + platformServos.getServoTwo().getPosition());
        telemetry.addData("Linked Pos", platformServos.getPosition());
        telemetry.update();
    }
}
