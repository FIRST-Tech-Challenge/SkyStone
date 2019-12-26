package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Autonomous ONE will do simple operation regarding the grabbing of the score depot and parking from the depot side
as opposed to the human player side.
 */

@Autonomous(name = "auto1", group = "autonomous")
public class auto1 extends LinearOpMode {

    public DcMotor TL;
    public DcMotor TR;
    public DcMotor BL;
    public DcMotor BR;

    public Servo hooker;

    @Override
    public void runOpMode() throws InterruptedException {

        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        TL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        hooker = hardwareMap.get(Servo.class, "hook");

        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime backgroundTime = new ElapsedTime();

        waitForStart();

        while(opModeIsActive()) {
            //Go forward
            backgroundTime.reset();
            drive(1, backgroundTime.seconds(), 5);
            dropDL();

            //Go backwards
            backgroundTime.reset();
            drive(2, backgroundTime.seconds(), 5);
            raiseDL();

        }
    }

    public void dropDL() {
        hooker.setPosition(0.9);
    }

    public void raiseDL() {
        hooker.setPosition(0.1);
    }

    public void drive(int cas3, double timeStart, double duration) {
        switch (cas3)
        {
            case 1 :
                /*
                drive forward for x time
                 */
                while (timeStart<=time){
                    TL.setPower(0.5);
                    TR.setPower(0.5);
                    BL.setPower(0.5);
                    BR.setPower(0.5);
                }
            case 2 :
                /*
                drive backward for x time
                 */
                while(timeStart<=time){
                    TL.setPower(-0.5 * -1);
                    TR.setPower(0.5 * -1);
                    BL.setPower(-0.5 * -1);
                    BR.setPower(0.5 * -1);
                }
            case 3 :
                /*
                Turn ninety degrees right
                 */
                while (timeStart<=time){
                    TL.setPower(-0.5);
                    BL.setPower(-0.5);
                    TR.setPower(0.5 * -1);
                    BR.setPower(0.5 * -1);
                }
            case 4 :
                /*
                Turn ninety degrees left
                 */
                while (timeStart<=time) {
                    TL.setPower(-0.5 * -1);
                    BL.setPower(-0.5 * -1);
                    TR.setPower(0.5);
                    BR.setPower(0.5);
                }
        }
    }


}
