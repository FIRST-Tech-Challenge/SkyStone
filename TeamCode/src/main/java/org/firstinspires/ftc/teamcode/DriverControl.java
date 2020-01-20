package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@TeleOp(name="DriverControl", group="Linear Opmode")
public class DriverControl extends Movement {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpModeImpl() {
        // TODO Set up frontServo in movement

        leftConstruction.resetDeviceConfigurationForOpMode();


        waitForStart();
        runtime.reset();


        while(opModeIsActive()) {
            while(true) {
                // Gamepad 1 controls:

                // Left trigger - to move left sideways
                goLeft(-gamepad1.left_trigger, 0);

                // Right trigger - to move right sideways
                goRight(-gamepad1.right_trigger, 0);

                // Left stick y - to go forward or backward
                double drive = -(-gamepad1.left_stick_y);

                // Right stick x - to turn left or right
                double turn  = (-gamepad1.right_stick_x);

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
                arm.setPower(-gamepad2.left_stick_y*0.66);

                // left bumper - to close claw (front servo)
                if (gamepad2.left_bumper) {

                    //frontServo.setPosition(0.4);
                    //sleep(100);
                    frontServo.setPosition(0.4);
                    telemetry.addData("front servo open", "clawposition: 0.4" );
                }

                // right bumper - to open claw (front servo)
                if (gamepad2.right_bumper) {

                    frontServo.setPosition(0.0);
                    telemetry.addData("front servo closed", "clawposition: 0.1" );
                }

                // a - arm target position

//                if (gamepad2.a) {
  //                  arm.setTargetPosition(1);
    //                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      //              arm.setPower(0);
        //            sleep(100);
          //      }

                // x - to move back servo down
                if (gamepad2.x) {

                    leftConstruction.setPosition(0.35);
                    rightConstruction.setPosition(0.43);
                    telemetry.addData("back servos down", "servoposition: 0,0" );
                }

                // y - to move back servo up
                if (gamepad2.y) {
                    leftConstruction.setPosition(1);
                    rightConstruction. setPosition(1);
                    telemetry.addData("back servos up", "servoposition: 0.95" );
                }

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();
            }
        }

    }
}

