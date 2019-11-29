package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class AutononousUC2 {

    public Intake intake;
    public Arm arm;
    public Chassis chassis;

    public AutononousUC2(HardwareMap hardwareMap) {

        intake = new Intake(hardwareMap);
        arm = new Arm(hardwareMap);
        chassis = new Chassis(hardwareMap);
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
