package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

@TeleOp(name = "AutonomousUC3", group = "Autonomous")
public class AutonomousUC3 extends LinearOpMode {

    public Intake intake;
    public Arm arm;
    public Chassis chassis;

    double distance = 0.5;
    double targetAngle = 0;
    double turn = 0;
    double power = 0.5;

    ElapsedTime AutonomousTimeOut = new ElapsedTime();

    @Override

    public void runOpMode() throws InterruptedException {

        intake = new Intake(hardwareMap);
        arm = new Arm(hardwareMap);
        chassis = new Chassis(hardwareMap);

        //Robot starts on A2
        waitForStart();
        while (opModeIsActive() && AutonomousTimeOut.milliseconds()<30000){

            //Move robot in between A5 and A6
            chassis.runDistance(distance, targetAngle, turn, power);
        }


    }
}
