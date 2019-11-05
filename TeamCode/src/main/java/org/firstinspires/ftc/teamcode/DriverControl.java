package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@TeleOp(name="DriverControl", group="Linear Opmode")
public class DriverControl extends Movement {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpModeImpl() {
        // TODO Set up frontServo in movement

        waitForStart();
        runtime.reset();


        while(opModeIsActive()) {
            while(true) {
                // Gamepad 1 controls:

                // Left trigger - to move left sideways
                goLeft(-gamepad1.left_trigger, 0, "Going left");

                // Right trigger - to move right sideways
                goRight(-gamepad1.right_trigger, 0, "Going right");

                // Left stick y - to go forward or backward
                double drive = -gamepad1.left_stick_y;

                // Right stick x - to turn left or right
                double turn  = -gamepad1.right_stick_x;

                // to drive and turn left?
                double leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
                leftfront.setPower(leftPower);
                leftback.setPower(leftPower);

                // to drive and turn right?
                double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
                rightfront.setPower(rightPower);
                rightback.setPower(rightPower);

                // Gamepad 2 Controls

                // Left stick y - to move arm up or down
                arm.setPower(-gamepad2.left_stick_y * 0.35);

                // left bumper - to open claw (front servo)
                if (gamepad2.left_bumper) {
                    // TODO: setpower / duration / name frontServo from hw config

                    //frontServo.setPosition(0.4);
                    //sleep(100);
                    frontServo.setPosition(0.4);
                }

                // right bumper - to close claw (front servo)
                if (gamepad2.right_bumper) {
                    // TODO: setpower / duration / name frontServo from hw config

                    //frontServo.setPosition(0.8);
                    //sleep(100);
                    frontServo.setPosition(0.04);
                }

                if (gamepad2.a)  {
                    arm.setTargetPosition(1);
                    arm.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
                    arm.setPower(0);
                    sleep(100);

                }

                // x - to move back servo down
                if (gamepad2.x) {
                    // TODO: setpower / duration
                    backServo.setPosition(0.4);
                }

                // y - to move back servo up
                if (gamepad2.y) {
                    // TODO: setpower / duration
                    backServo.setPosition(0.95);
                }

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();
            }
        }

    }
}
