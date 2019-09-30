package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="AutonTest", group="LinearOpMode")
public class AutonTest extends LinearOpMode {
    
    HardwareRobot robot = new HardwareRobot();
    
    public void robotInit() {
        robot.init(hardwareMap);
       
    }
    @Override
    public void runOpMode() {
        robotInit();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // wait for the game to start (driver presses PLAY)
        waitForStart();
          moveYWX(0,0,0.4);
       sleep(550);
       
       
         moveYWX(0, 0, 0);
       sleep(500);
       
        moveYWX(-0.4, 0, 0);
       sleep(400);
       
        moveYWX(0, 0, 0);
       sleep(500);
       
      moveYWX(0.4, 0, 0);
       sleep(950);
       
        moveYWX(0, 0, 0);
       sleep(500);
       
        moveYWX(-0.4, 0, 0);
       sleep(2000);
       
        moveYWX(0, 0, 0);
       sleep(500);
       
         moveYWX(0,0,0.4);
       sleep(2000);
       
       
         moveYWX(0, 0, 0);
       sleep(500);
       
        moveYWX(-0.4,0,0);
       sleep(500);
       
       
         moveYWX(0, 0, 0);
       sleep(500);
       
       moveYWX(0,0,-0.6);
       sleep(3200);
       
       
         moveYWX(0, 0, 0);
       sleep(500);
       
        moveYWX(-0.4,0,0);
       sleep(350);
       
       
         moveYWX(0, 0, 0);
       sleep(500);
      
       
       
     
    }
    
    public void moveYWX(double forward, double turn, double strafe) {
        robot.leftFront.setPower(forward + turn + strafe);
        robot.rightFront.setPower(forward - turn - strafe);
        robot.leftBack.setPower(forward + turn - strafe);
        robot.rightBack.setPower(forward - turn + strafe);
    }

    
   
}
