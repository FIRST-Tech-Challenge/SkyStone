package org.firstinspires.ftc.teamcode.mason_wu;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Omniwheel", group = "Linear Opmode")

public class Omniwheel_TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor R_INTAKE = null;
    private DcMotor L_INTAKE = null;

    double speed = 1;
    double intake_speed = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        R_INTAKE = hardwareMap.get(DcMotor.class, "R_INTAKE");
        L_INTAKE = hardwareMap.get(DcMotor.class, "L_INTAKE");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        R_INTAKE.setDirection(DcMotor.Direction.REVERSE);
        L_INTAKE.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double LFPower;
            double RFPower;
            double LBPower;
            double RBPower;
            double rotate = 0;

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;


            if(gamepad1.left_bumper) {
                speed -= 0.01;
                sleep(100);
            }


            if(gamepad1.right_bumper) {
                speed += 0.01;
                sleep(100);
            }


            intake_speed -= 0.1*gamepad1.left_trigger;

            intake_speed += 0.1*gamepad1.right_trigger;

            if(gamepad1.dpad_left){
                rotate += 0.3;
                if(rotate > 1){
                    rotate = 1;
                }
                if(rotate < -1){
                    rotate = -1;
                }
                sleep(100);

            }

            if(gamepad1.dpad_right){
                rotate -= 0.3;
                if(rotate > 1){
                    rotate = 1;
                }
                if(rotate < -1){
                    rotate = -1;
                }
                sleep(100);

            }

            if(gamepad1.x){
                R_INTAKE.setPower(0.5*intake_speed);
                L_INTAKE.setPower(0.5*intake_speed);
            }
            if(gamepad1.b){
                R_INTAKE.setPower(0);
                L_INTAKE.setPower(0);
            }

            LFPower    = Range.clip(speed*(drive + turn - rotate), -1.0, 1.0) ;
            LBPower    = Range.clip(speed*(drive - turn -rotate), -1.0, 1.0) ;
            RFPower   = Range.clip(speed*(drive - turn + rotate), -1.0, 1.0) ;
            RBPower   = Range.clip(speed*(drive + turn + rotate), -1.0, 1.0) ;

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);


        }

    }
}

