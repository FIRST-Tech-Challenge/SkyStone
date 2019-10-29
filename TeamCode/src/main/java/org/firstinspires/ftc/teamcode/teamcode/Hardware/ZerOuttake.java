package org.firstinspires.ftc.teamcode.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ZerOuttake{

    public Servo pushBlock;
    public Servo hookRight;
    public Servo hookLeft;
    public CRServo rightVex;
    public CRServo leftVex;
    public DcMotor liftRight;
    public DcMotor liftLeft;

    boolean hookClosed;

    LinearOpMode linOpMode;
    OpMode opMode;
    ElapsedTime time = new ElapsedTime();

    //1.055 Inches Base, Foundation is 2.25 inches

    static double HOOKDOWN = -1.0;
    static double HOOKUP = 1.0;
    private boolean toggled = false;

    public void initOuttake(OpMode opMode)
    {
        time.reset();
        this.opMode = opMode;

        pushBlock = opMode.hardwareMap.servo.get("PB");
        rightVex = opMode.hardwareMap.crservo.get("ROut");
        leftVex = opMode.hardwareMap.crservo.get("LOut");
        liftLeft = opMode.hardwareMap.dcMotor.get("LLift");
        liftRight = opMode.hardwareMap.dcMotor.get("RLift");
        hookLeft = opMode.hardwareMap.servo.get("LHook");
        hookRight = opMode.hardwareMap.servo.get("RHook");

        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opMode.telemetry.addData("Success", "Outtake Initialized");
        opMode.telemetry.update();

    }

    public void zeroTeleOut() {

            if (Math.abs(opMode.gamepad2.right_stick_y) >=.075) {
                rightVex.setPower(opMode.gamepad2.right_stick_y / 2);
                leftVex.setPower(-opMode.gamepad2.right_stick_y / 2);
            }
            else {
                rightVex.setPower(0);
                leftVex.setPower(0);
            }

            if (Math.abs(opMode.gamepad2.left_stick_y) > .05) {
                if ((liftRight.getCurrentPosition() + liftLeft.getCurrentPosition()) /2
                        == 0 && -opMode.gamepad2.left_stick_y > 0.5 ||
                        (liftRight.getCurrentPosition() + liftRight.getCurrentPosition()) / 2 ==
                                -6500 && -opMode.gamepad2.left_stick_y < -0.5) {
                    liftRight.setPower(opMode.gamepad2.left_stick_y);
                    liftLeft.setPower(opMode.gamepad2.left_stick_y);
                }
            } else {
                liftRight.setPower(0);
                liftLeft.setPower(0);
            }

            if (opMode.gamepad2.x) {
                if (hookClosed) {
                    pushBlock.setPosition(1);
                    hookClosed = !hookClosed;
                    while(pushBlock.getPosition() != 1) {}
                } else if (!hookClosed) {
                    pushBlock.setPosition(.5);
                    hookClosed = !hookClosed;
                    while(pushBlock.getPosition() != .5) {}
                }
            }

            hookToggle();
    }

    public void hookToggle() {
        if(!toggled && opMode.gamepad2.y)
        {
            toggled = true;

            hookLeft.setPosition(HOOKUP);
            hookRight.setPosition(HOOKDOWN);

            while(hookRight.getPosition() != HOOKDOWN &&
                    hookLeft.getPosition() != HOOKUP) {}
        }
        else if(toggled && opMode.gamepad2.y)
        {
            toggled = false;

            hookLeft.setPosition(HOOKDOWN);
            hookRight.setPosition(HOOKUP);

            time.reset();
            while(hookRight.getPosition() != HOOKUP &&
                    hookLeft.getPosition() != HOOKDOWN) {}
        }
    }
}
