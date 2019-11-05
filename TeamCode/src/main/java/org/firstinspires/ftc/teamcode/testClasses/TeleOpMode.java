package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


import java.util.HashMap;

@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        telemetry.addData("Init", "v:1.0");
        waitForStart();
            Controller controller = new Controller(gamepad1);
            robot.run(controller);
            /*
            HashMap<String, Integer> chassisMotorPositions = chassis.getMotorPositions();
            for(HashMap.Entry<String, Integer> chassisMotorPosition : chassisMotorPositions.entrySet()){
               telemetry.addData(chassisMotorPosition.getKey(), chassisMotorPosition.getValue());
            }
            */
            telemetry.addData("arm motor: ", armMotor.getCurrentPosition());
            telemetry.update();
        }


    public void runHook(Hook hook, Controller controller) {

    }

    public void runChassis(Chassis chassis, Controller controller) {

        //chassis.runChassis(angle, turn, power);
    }
}  
