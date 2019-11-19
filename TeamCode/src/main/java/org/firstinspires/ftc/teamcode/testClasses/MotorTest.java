package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autoRes.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Motor Test", group = "Teleop")
public class MotorTest extends LinearOpMode {
    long timeTo = System.currentTimeMillis();

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Chassis chassis = new Chassis(hardwareMap);
        chassis.reverseMotors(new DcMotor[]{chassis.frontRight, chassis.backRight});
        robot.setChassis(chassis);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                robot.chassis.runRotations(-2,1);
                while(gamepad1.a){
                    telemetry.addData("avg motor err: ",robot.chassis.getAverageMotorError());
                    telemetry.update();
                }
            }
            //Other unit test code if you want
            telemetry.update();
        }
    }
}