package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Hormuz + Eric <3", group="Beta")
public class Hormuz extends LinearOpMode {
    
    HardwareRobot robot = new HardwareRobot();
    Servo leftArm = null;
    Servo rightArm = null;
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        
        rightArm = hardwareMap.get(Servo.class, "right_arm");
        leftArm = hardwareMap.get(Servo.class, "left_arm");
        
        rightArm.setPosition(0.05);
        //leftArm.setPosition(0.5);
        
        telemetry.addData("Initialized", "oh yeah");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive()) {
            double leftStickY = -gamepad1.left_stick_y;
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = gamepad1.right_stick_x;
            
            mecDrive(-leftStickY,rightStickX,-leftStickX);
            
            
            while (gamepad1.left_bumper) {
                mecDrive(0,0.2,0);
            }
            
            while (gamepad1.right_bumper) {
                mecDrive(0,-0.2,0);
            }
            
            if (gamepad2.x) {
                rightArm.setPosition(0.9);
            }
            
            if (gamepad2.y) {
                rightArm.setPosition(0.5);
            }
            
            if (gamepad2.b) {
                leftArm.setPosition(0.05);
            }
            
            if (gamepad2.a) {
                leftArm.setPosition(0.35);
            }
            

            
            robot.leftIntake.setPower(+gamepad1.left_trigger - gamepad1.right_trigger);
            robot.rightIntake.setPower(+gamepad1.left_trigger - gamepad1.right_trigger);
            
        }
    }
    
    public void mecDrive(double forward, double turn, double strafe) {
        robot.leftFront.setPower(Range.clip(forward + turn + strafe,-0.5,0.5));
        robot.rightFront.setPower(Range.clip(forward - turn - strafe,-0.5,0.5));
        robot.leftBack.setPower(Range.clip(forward + turn - strafe,-0.5,0.5));
        robot.rightBack.setPower(Range.clip(forward - turn + strafe,-0.5,0.5));
    }
    
}
