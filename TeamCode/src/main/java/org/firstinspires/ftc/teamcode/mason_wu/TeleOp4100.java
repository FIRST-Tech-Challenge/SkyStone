package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp 4100-mwu", group="Linear Opmode")
@Disabled
public class TeleOp4100 extends LinearOpMode{

    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            double FLPower;
            double FRPower;
            double BLPower;
            double BRPower;
            double rotate = 0;
            double speed = 0.8;
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;


            if(gamepad1.dpad_up){
                speed += 0.2;
                if(speed < 0){
                    speed = 0;
                }
            }
            if(gamepad1.dpad_down){
                speed -= 0.2;
                if(speed < 0){
                    speed = 0;
                }
            }
            if(gamepad1.left_bumper){
                rotate += 0.3;
                if(rotate > 1){
                    rotate = 1;
                }
                if(rotate < -1){
                    rotate = -1;
                }
            }

            if(gamepad1.right_bumper){
                rotate -= 0.3;
                if(rotate > 1){
                    rotate = 1;
                }
                if(rotate < -1){
                    rotate = -1;
                }
            }

            FLPower    = Range.clip(speed * (drive + turn - rotate), -1.0, 1.0) ;
            FRPower   = Range.clip(speed * (drive - turn + rotate), -1.0, 1.0) ;
            BLPower    = Range.clip(speed * (drive - turn - rotate), -1.0, 1.0) ;
            BRPower   = Range.clip(speed * (drive + turn + rotate), -1.0, 1.0) ;

            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);

            telemetry.addData("Motors", "FL (%.2f), FR (%.2f)", FLPower, FRPower);
            telemetry.addData("Motors", "BL (%.2f), BR (%.2f)", BLPower, BRPower);
            telemetry.update();
        }
    }

}


