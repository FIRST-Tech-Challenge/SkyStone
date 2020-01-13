package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.MaccabotV2;

public class TeleOpV2 extends OpMode {

    private MaccabotV2 robot;

    @Override
    public void init() {
        robot = new MaccabotV2(this);
        robot.initialize(false);
    }

    @Override
    public void start() {
        telemetry.clearAll();
        telemetry.addLine("4466 TELEOP v2 | ACTIVE");
        robot.drawTelemetry(); // puts up debug values for use during practice
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.drive.arcadeMecanumDrive(gamepad1.left_stick_x,
                -gamepad1.left_stick_y, -gamepad1.right_stick_x);

        robot.intake.runIntake(gamepad1.right_trigger - gamepad1.left_trigger);

        robot.lift.runLift(gamepad2.right_stick_y);

        robot.lift.runRack(gamepad2.left_stick_y);

        robot.lift.runChad(gamepad2.right_bumper, gamepad2.left_bumper);
    }
}
