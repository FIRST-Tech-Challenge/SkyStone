package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Gyro2", group="LinearOpMode")
@Disabled
public class Gyro2 extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    private BNO055IMU imu;

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
    while (opModeIsActive()) {

        //driveYWT(0.2,0,1);
        driveYWT(0.1,30,2);
        moveYW(0,0);
    } 


 moveYW(0,0);



    }

   public void moveYW(double forward, double turn) {
        robot.leftFront.setPower(forward + turn);
       robot.rightFront.setPower(forward - turn);
       robot.leftBack.setPower(forward + turn);
       robot.rightBack.setPower(forward - turn);
   }


   public void driveYWT(double forward, double turn, double sec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        telemetry.addData("time", timer.seconds());
        telemetry.update();
        while (opModeIsActive()) {


            if (timer.seconds() > sec) {
                moveYB(0, 0);
            } 
            else {
            moveYB(forward, turn);
        }
            
        }
   }

   // NEW FUNCTIONS

    public void moveYB(double forward, double bearing) {
        Orientation angles = imu.getAngularOrientation();
        telemetry.addData("imu", angles.toString());
        telemetry.update();
        double heading = angles.firstAngle;
        moveYW(forward, (heading-bearing) * 0.02);
    }

    }

