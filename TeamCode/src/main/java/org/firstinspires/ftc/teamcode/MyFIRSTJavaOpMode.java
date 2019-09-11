package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MyFIRSTJavaOpMode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorTest1;
    private DcMotor motorTest2;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange; private Servo servoTest;
    double left;
    double right;
    double drive;
    double turn;
    double max;

    @Override
    public void runOpMode() {
//        imu = hardwareMap.get(Gyroscope.class, "imu");
        //text
        motorTest1 = hardwareMap.get(DcMotor.class, "left_drive");
        motorTest2 = hardwareMap.get(DcMotor.class, "right_drive");
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
//        servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {
//            tgtPower = -this.gamepad1.left_stick_y;
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;
//            turn = 0;

            // Combine drive and turn for blended motion.
            left  = -( drive + turn );
            right = (drive - turn);

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            motorTest1.setPower(right);
            motorTest2.setPower(left);
            telemetry.addData("Target Power 1", left);
            telemetry.addData("Target Power 2", right);
            telemetry.addData("Motor Power", motorTest1.getPower());
            telemetry.addData("Motor Power", motorTest2.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
