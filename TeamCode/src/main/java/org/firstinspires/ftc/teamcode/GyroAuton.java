package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "GyroAuton", group = "LinearOpMode")
@Disabled
public class GyroAuton extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    Servo leftArm = null;
    Servo rightArm = null;
    
    private ElapsedTime scanTimer = new ElapsedTime();
    
    private BNO055IMU imu;
    double lastZ = 0;
    int turns = 0;
    
    double MID_SERVO = 0.5 ;
    double ARM_UP_POWER = 0.45 ;
    double ARM_DOWN_POWER = -0.45 ;

    public void robotInit() {
        robot.init(hardwareMap);
        
        leftArm  = hardwareMap.get(Servo.class, "left_arm");
        rightArm = hardwareMap.get(Servo.class, "right_arm");
        leftArm.setPosition(-0.5);
       
        
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
        scanTimer.reset();
        



              /* 1 + 4 SkyStone
        driveYBT(0, 0, 0.2, .25);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(0.2,0,0,.5);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(-0.2,0,0,2);
        
        
        driveYBT(0,0,0,0.5);
        
        leftArm.setPosition(0.3);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(0.2,0,0,1.5);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(0, 0, -0.6, 1.5);
        
        driveYBT(0,0,0,0.5);
        
        leftArm.setPosition(-0.9);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(0, 0, 0.6, 1.75);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(0, 0, 0.3, .5);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(0, 0, -0.2, .5);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(0.2, 0, 0, 1);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(-0.2,0,0,2);
        
        driveYBT(0,0,0,0.5);
        
        
        
        leftArm.setPosition(0.3);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(0.2, 0, 0, 1);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(0, 0, -0.6, 2);
        
        driveYBT(0,0,0,0.5);
        
        leftArm.setPosition(-0.9);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(0, 0, 0.2, .5);
        
        driveYBT(0,0,0,0.5);
        
        driveYBT(0, 0, 0.6, .5);
        
        driveYBT(0,0,0,0.5);
        moveYW(0, 0, 0); */
        
        // 2 + 4 objects
                    driveYBT(0, 0, -0.2, 1);

                                driveYBT(0, 0, 0, 0.35);

                                driveYBT(0.2, 0, 0, .5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(-0.2, 0, 0, 2);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(0.3);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, 1.5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.6, 1.5);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(-0.9);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.6, 1.75);

                                driveYBT(0, 0, 0, 0.5);
                
                                driveYBT(0, 0, 0.3, 1);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.2, 1.5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, 1);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(-0.2, 0, 0, 2);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(0.3);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0.2, 0, 0, 1);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, -0.6, 1.75);

                                driveYBT(0, 0, 0, 0.5);

                                leftArm.setPosition(-0.9);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.2, .5);

                                driveYBT(0, 0, 0, 0.5);

                                driveYBT(0, 0, 0.6, .5);

                                driveYBT(0, 0, 0, 1);
                                
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