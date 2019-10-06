package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Forward_Alan extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    double power = 0.5;
    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("Left_Motor");
        rightMotor = hardwareMap.dcMotor.get("Right_Motor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        sleep(2000);

        power = 0.0;

        leftMotor.setPower(power);
        rightMotor.setPower(power);

    }
}
