package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Individual Encoder Test", group="Test")
@Disabled
public class IndividualEncoderTest extends LinearOpMode {

    private DcMotor front_left, front_right, back_left, back_right, intake_left, intake_right;

    public void runOpMode() throws InterruptedException {
        front_left = hardwareMap.dcMotor.get("front_left"); // Port 0
        front_right = hardwareMap.dcMotor.get("front_right"); // Port 1
        back_left = hardwareMap.dcMotor.get("back_left"); // Port 2
        back_right = hardwareMap.dcMotor.get("back_right"); // Port 3
        intake_left = hardwareMap.dcMotor.get("intake_left");
        intake_right = hardwareMap.dcMotor.get("intake_right");


        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                front_right.getCurrentPosition(),
                front_left.getCurrentPosition(),
                back_left.getCurrentPosition(),
                back_right.getCurrentPosition());
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.a) {
                back_left.setPower(0.25);
                back_left.setTargetPosition(500);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (gamepad1.b) {
                front_left.setPower(0.25);
                front_left.setTargetPosition(-500);
                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.x) {
                front_right.setPower(0.25);
                front_right.setTargetPosition(-500);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.y) {
                back_right.setPower(0.25);
                back_right.setTargetPosition(500);
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.a) {
                back_left.setPower(0.25);
                back_left.setTargetPosition(-500);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.b) {
                front_left.setPower(0.25);
                front_left.setTargetPosition(-500);
                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.x) {
                front_right.setPower(0.25);
                front_right.setTargetPosition(-500);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.y) {
                back_right.setPower(0.25);
                back_right.setTargetPosition(-500);
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

        /*back_left.setPower(0.25);
        back_left.setTargetPosition(500);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        back_left.setPower(0);


        front_left.setPower(0.25);
        front_left.setTargetPosition(500);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        front_left.setPower(0);



        front_right.setPower(0.25);
        front_right.setTargetPosition(500);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        front_right.setPower(0);


        back_right.setPower(0.25);
        back_right.setTargetPosition(500);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        back_right.setPower(0);*/

            telemetry.addData("front-left", front_left.getCurrentPosition());
            telemetry.addData("back-left", back_left.getCurrentPosition());
            telemetry.addData("front-right", front_right.getCurrentPosition());
            telemetry.addData("back-right", back_right.getCurrentPosition());

            telemetry.update();


        /*back_left.setPower(0.25);
        back_left.setTargetPosition(-500);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        back_left.setPower(0);


        front_left.setPower(0.25);
        front_left.setTargetPosition(-500);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        front_left.setPower(0);


        front_right.setPower(0.25);
        front_right.setTargetPosition(-500);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        front_right.setPower(0);


        back_right.setPower(0.25);
        back_right.setTargetPosition(-500);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        back_right.setPower(0);
         */

            telemetry.addData("front-left", front_left.getCurrentPosition());
            telemetry.addData("back-left", back_left.getCurrentPosition());
            telemetry.addData("front-right", front_right.getCurrentPosition());
            telemetry.addData("back-right", back_right.getCurrentPosition());

            telemetry.update();

        }
    }
}
