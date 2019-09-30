package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "GyroAuton", group = "LinearOpMode")
public class GyroAuton extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    private BNO055IMU imu;
    double lastZ = 0;
    int turns = 0;

    public void robotInit() {
        robot.init(hardwareMap);

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
    }
    // @Override
    public void runOpMode() {
        robotInit();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // wait for the game to start (driver presses PLAY)
        waitForStart();



        driveYBT(0.2, 180, 0, 1.75);
        driveYBT(0.1, 45, 0, 3);
        //driveYBT(0, 0, 0.4, 1);
        //  driveYBT(0, 0, .4, 3);
        moveYW(0, 0, 0);

    }

    public void moveYW(double forward, double turn, double strafe) {
        robot.leftFront.setPower(forward + turn + strafe);
        robot.rightFront.setPower(forward - turn - strafe);
        robot.leftBack.setPower(forward + turn - strafe);
        robot.rightBack.setPower(forward - turn + strafe);
    }

    public void moveYB(double forward, double bearing, double strafe) {
        Orientation angles = imu.getAngularOrientation();
        moveYW(forward, (heading() - bearing) * 0.01, strafe);
    }

    public void driveYBT(double forward, double bearing, double strafe, double sec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {
            if (timer.seconds() > sec) break;
            moveYB(forward, bearing, strafe);
        }
    }

    public double heading() {
        Orientation angles = imu.getAngularOrientation();
        double imuZ = angles.firstAngle;
        // see if cross boundary from plus to minus or vice-versa
        if (lastZ > 140 && imuZ < -140) turns++;
        if (lastZ < -140 && imuZ > 140) turns--;
        lastZ = imuZ;
        return imuZ + turns * 360;
    }

}
