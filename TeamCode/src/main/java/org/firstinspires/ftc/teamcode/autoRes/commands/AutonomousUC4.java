package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

@TeleOp (name = "AutonomousUC4" , group = "Autonomous")
public class AutonomousUC4 extends LinearOpMode {

    public Intake intake;
    public Arm arm;
    public Chassis chassis;

    double distance = 1;
    double targetAngle = 0;
    double turn = 0;
    double power = 0.75;

    @Override

    public void runOpMode() throws InterruptedException {

        intake = new Intake(hardwareMap);
        arm = new Arm(hardwareMap);
        chassis = new Chassis(hardwareMap);

        //robot will start on
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
