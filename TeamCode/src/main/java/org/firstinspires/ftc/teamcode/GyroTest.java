package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="GyroTest", group="LinearOpMode")
public class GyroTest extends LinearOpMode {
    
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    
    private BNO055IMU imu;
    
    public void robotInit() {
        
        
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
    }
    @Override
    public void runOpMode() {
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // wait for the game to start (driver presses PLAY)
        waitForStart();
        
       driveYBT(0.4, 0, 2); // drive heading 0 for 2 seconds
       driveYBT(0.1, 45, 3); // drive heading 45 for 3 seconds
       driveYBT(0.4, -20, 4); // drive heading -20 for 4 seconds
       moveYWX(0, 0);
    }
    
    public void moveYWX(double forward, double turn) {
        leftDrive.setPower(forward + turn);
        rightDrive.setPower(forward - turn);
    }
    
    // start the robot moving forward (Y) to a given bearing
    public void moveYB(double forward, double bearing) {
         Orientation angles = imu.getAngularOrientation();
         double heading = angles.firstAngle;
         moveYWX(forward, (bearing-heading) * 0.02);
    }
    
    // drive the robot forward (Y) and/or turning (W) for seconds (T)
    public void driveYBT(double forward, double bearing, double sec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {
            if (timer.seconds() > sec) break;
            moveYB(forward, bearing);
        }
        
    }
}
