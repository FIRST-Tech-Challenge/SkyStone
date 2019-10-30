package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Spool Mover", group = "Sensor")
//@Disabled
public class SpoolMover extends LinearOpMode {

    private DcMotor _spool;

    @Override public void runOpMode()
    {
        _spool = hardwareMap.get(DcMotor.class, "spool");
        _spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        telemetry.log().add("...started...");

        while (opModeIsActive()) {

            if(gamepad2.left_trigger > 0){
                _spool.setDirection(DcMotorSimple.Direction.REVERSE);
                _spool.setPower(1);
            }
            else if(gamepad2.right_trigger > 0){
                _spool.setDirection(DcMotorSimple.Direction.FORWARD);
                _spool.setPower(1);
            }
            else {

                _spool.setPower(0);
            }
        }

    }
}
