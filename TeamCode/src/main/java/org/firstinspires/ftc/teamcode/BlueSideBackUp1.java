package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "BlueSideBackUpStoneFacing", group = "Concept")
//@Disabled
public class BlueSideBackUp1 extends LinearOpMode {
    
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

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
       
        robotInit();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
       
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

      
                                driveYBT(-0.2, 0, 0, 0.5);
                                driveYBT(0, 0, 0, 0.5);
                                 driveYBT(0, 0, 0.3, 1.8);
                                  driveYBT(0, 0, 0, 0.5);
                                 driveYBT(0.3, 0, 0, 0.5);

                               
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
