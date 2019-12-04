package org.firstinspires.ftc.teamcode.Miscellaneous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Motor Check (beta)", group = "Linear OpMode")
public class MotorCheck extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor drive_FL, drive_FR, drive_RL, drive_RR;
    private DcMotor intake_L, intake_R;
    private Servo latch_L, latch_R;

    private String testStatus = "Preparing to run tests...";

    @Override
    public void runOpMode() {
        drive_FL = hardwareMap.get(DcMotor.class, "front left");
        drive_FR = hardwareMap.get(DcMotor.class, "front right");
        drive_RL = hardwareMap.get(DcMotor.class, "rear left");
        drive_RR = hardwareMap.get(DcMotor.class, "rear right");

        intake_L = hardwareMap.get(DcMotor.class, "left intake");
        intake_R = hardwareMap.get(DcMotor.class, "right intake");

        latch_L = hardwareMap.get(Servo.class, "left servo");
        latch_R = hardwareMap.get(Servo.class, "right servo");

        drive_FR.setDirection(DcMotor.Direction.REVERSE);
        drive_RL.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // Begin tests
        telemetry.addData("Status", testStatus);
        telemetry.update();

        sleep(1000);

        // Intake test
        for (int i = 0; i <= 1; i += 0.1) {
            intake_L.setPower(i);
            intake_R.setPower(i);

            sleep(500);
        }

        // Servo test
        for (int i = 0; i <= 1; i += 0.05) {
            latch_L.setPosition(i);
            latch_R.setPosition(1 - i);

            sleep(500);
        }

        // Drive train test






    }

}