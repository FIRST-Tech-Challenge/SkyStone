package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.motion.Kicker;

@TeleOp(name="Permeet's First TeleOp", group="Permeet")
public class PermeetTeleop extends LinearOpMode{

    RobotHardware robot = new RobotHardware();
    Kicker kicker = new Kicker();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap, telemetry);
        telemetry.addData( "Hi there ", "Permeet" );
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            if (gamepad1.right_trigger > 0 ) {
                kicker.moveKicker(robot,gamepad1.right_trigger);

            } else {
                robot.kicker.setPosition(robot.KICKER_START);
            }


        }
    }
}
