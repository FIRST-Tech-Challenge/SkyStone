package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MecanumDrive", group="Beta")
public class MecanumDrive extends LinearOpMode {
    
    HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        
        waitForStart();
        
        while (opModeIsActive()) {
            double leftStickY = -gamepad1.left_stick_y;
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = gamepad1.right_stick_x;
            
            moveYWX(jy,rightStickX,leftStickX);
            
        }
    }
    
    public void moveYW(double forward, double turn) {
        robot.leftBack.setPower(forward + turn);
        robot.rightBack.setPower(forward - turn);
    }
    
    public void moveYWX(double forward, double turn, double strafe) {
        robot.leftFront.setPower(forward + turn + strafe);
        robot.rightFront.setPower(forward - turn - strafe);
        robot.leftBack.setPower(forward + turn - strafe);
        robot.rightBack.setPower(forward - turn + strafe);
    }
    
}
