// package fJava;    >>> Specific to OnBot Java (I think)

package org.firstinspires.ftc.teamcode.Miscellaneous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Liberty TeleOp (beta)", group = "TeleOp")

public class LibertyTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor left_motor;
    private DcMotor right_motor;

    private DcMotor base_joint;
    private Servo mid_joint;
    private Servo claw;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializaed");
        telemetry.update();

        // The string in the hardwareMap needs to be the same as the configuration
        left_motor = hardwareMap.dcMotor.get("left_motor");
        right_motor = hardwareMap.dcMotor.get("right_motor");

        base_joint = hardwareMap.dcMotor.get("base");
        mid_joint = hardwareMap.servo.get("mid");
        claw = hardwareMap.servo.get("claw");

        // Put initialization blocks here.
        left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        base_joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Tank mode (each stick controls one side) for
                // drive train
                left_motor.setPower(-gamepad1.left_stick_y);
                right_motor.setPower(-gamepad1.right_stick_y);

                // Claw mechanism

                float LT = gamepad1.left_trigger;
                float RT = gamepad1.right_trigger;

                if (LT > 0 && RT == 0) {
                    base_joint.setPower(LT * -0.25);
                } else if (RT > 0 && LT == 0) {
                    base_joint.setPower(RT * 0.05);
                } else {
                    base_joint.setPower(-0.13);
                }

                boolean LB = gamepad1.left_bumper;
                boolean RB = gamepad1.right_bumper;

                if (LB) {
                    mid_joint.setPosition(1);
                } else if (RB) {
                    mid_joint.setPosition(0);
                }

                if (gamepad1.y) {
                    claw.setPosition(1);
                } else if (gamepad1.b) {
                    claw.setPosition(0);
                }

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("LTrigger", LT);
                telemetry.addData("RTrigger", RT);
                telemetry.addData("LBumper", LB);
                telemetry.addData("RBumper", RB);
                telemetry.update();
            }
        }
    }
}
