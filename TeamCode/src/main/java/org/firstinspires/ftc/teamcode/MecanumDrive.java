package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
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
            
            mecDrive(leftStickY,rightStickX,leftStickX);
            
            
            while (gamepad1.a) {
                mecDrive(0,0,0.4);
            }
            
            while (gamepad1.b) {
                mecDrive(0,0,-0.4);
            }
            
            robot.leftIntake.setPower(+gamepad1.left_trigger - gamepad1.right_trigger);
            robot.rightIntake.setPower(+gamepad1.left_trigger - gamepad1.right_trigger);
            
        }
    }
    
    public void mecDrive(double forward, double turn, double strafe) {
        robot.leftFront.setPower(Range.clip(forward + turn + strafe, -0.5, 0.5));
        robot.rightFront.setPower(Range.clip(forward - turn - strafe, -0.5, 0.5));
        robot.leftBack.setPower(Range.clip(forward + turn - strafe, -0.5, 0.5));
        robot.rightBack.setPower(Range.clip(forward - turn + strafe, -0.5, 0.5));
    }
    
}
