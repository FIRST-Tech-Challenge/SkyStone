package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class AutononousUC2 extends LinearOpMode {

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
            UC2Command(distance, targetAngle, turn, power);
        }
    }

    public void UC2Command(double distance, double targetAngle, double turn, double power){

        //Open wrist to open position
        intake.moveWristToHorizontal();
        //Move arm above the tray to get ready to pull it back
        arm.moveArm_aboveFoundationLevel();
        //Move Robot between C4 and C5
        chassis.runDistance( distance,  targetAngle,  turn,  power);
        //



    }

}
