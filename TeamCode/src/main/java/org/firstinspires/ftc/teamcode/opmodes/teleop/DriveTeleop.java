package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.opmodes.debuggers.TeleOpModeDebugger;
import org.firstinspires.ftc.teamcode.components.DriveSystem;


@TeleOp(name = "CompetitionTeleOp", group="TeleOp")
public class DriveTeleop extends TeleOpModeDebugger {
    private Controller controller1;
    private DriveSystem driveSystem;
    private boolean slowDrive;

    public DriveTeleop() {
        msStuckDetectLoop = 1000000000;
    }

    @Override
    public void init()
    {
        this.controller1 = new Controller(gamepad1);

        this.driveSystem = new DriveSystem(this);
        slowDrive = false;
    }


    @Override
    public void initialize() {

    }

    @Override
    public void run(){
        controller1.handle();

        float rx = controller1.gamepad.right_stick_x;
        float ry = controller1.gamepad.right_stick_y;
        float lx = controller1.gamepad.left_stick_x;
        float ly = controller1.gamepad.left_stick_y;

        driveSystem.drive(rx, ry, lx, ly, slowDrive);
    }

}
