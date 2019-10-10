package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "a_Testing_TeleOpControl")
public class a_Testing_TeleOpControl extends OpMode {

    //Declare Motors
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;

    private DcMotor slideMotor = null;

    //Declare Servos
    private CRServo boxFlipServo;
    private CRServo intakeServo;
    private CRServo grabberServo;
    private CRServo randomServo;
    //Declare misc variables
    private ElapsedTime TeleOP_runtime = new ElapsedTime();
    private double modifier_gamepad1Speed;


    //Run the code after 'Driver' hits INIT
    @Override
    public void init() {
        telemetry.addData("Initialising", "Started");
        telemetry.update();

        //Initialise hardware variables from configuration.
        //Phone configuration = "Release_v3"
        //This configuration can be edited without affecting release.

        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");


        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        grabberServo = hardwareMap.get(CRServo.class, "grabberServo");
        boxFlipServo = hardwareMap.get(CRServo.class, "boxFlipServo");
        randomServo = hardwareMap.get(CRServo.class, "randomServo");

        //slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");


        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        //slideMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Initialising", "Complete");
        telemetry.update();

    }

    //Run the loop after 'Driver' hits INIT until PLAY.
    @Override
    public void init_loop() {

    }

    //Run ONCE driver hits PLAY
    @Override
    public void start() {
        TeleOP_runtime.reset();
        modifier_gamepad1Speed = 0.5;
    }

    private boolean dpad1Pressed = false;
    private boolean dpad2Pressed = false;

    private double modifier_gamepad2Speed = 0.5;
    //Run the loop after 'Driver hits PLAY before STOP
    @Override
    public void loop() {
        // Change drive speed by changing the modifier

        // Dont change speed if buttons already pressed
        //if (!dpad1Pressed) {
            //XOR dpad_down/dpad_up. Keep gamepad speed within limits
         //   if ((!gamepad1.dpad_down && gamepad1.dpad_up) && (modifier_gamepad1Speed < 0.9)) {
           //     modifier_gamepad1Speed = modifier_gamepad1Speed + 0.1;
           // } else if ((gamepad1.dpad_down && !gamepad1.dpad_up) && (modifier_gamepad1Speed > 0.1)) {
             //   modifier_gamepad1Speed = modifier_gamepad1Speed - 0.1;
          //  }
        //}
        // Dont change speed if buttons already pressed
        if (!dpad2Pressed) {
            //XOR dpad_down/dpad_up. Keep gamepad speed within limits
            if ((gamepad2.dpad_up && !gamepad2.dpad_down) && modifier_gamepad2Speed < 0.9) {
                modifier_gamepad2Speed = modifier_gamepad2Speed + 0.1;
            } else if ((gamepad2.dpad_down && !gamepad2.dpad_up) && modifier_gamepad2Speed > 0.1) {
                modifier_gamepad2Speed = modifier_gamepad2Speed - 0.1;
            }
        }
        //Check for gamepad buttons to be pressed or not for next loop
        dpad1Pressed = (gamepad1.dpad_up || gamepad1.dpad_down);
        dpad2Pressed = (gamepad2.dpad_up || gamepad2.dpad_down);

        double leftInput = gamepad2.left_trigger + (gamepad2.left_bumper ? -1 : 0);
        double rightInput = gamepad2.right_trigger + (gamepad2.right_bumper ? -1 : 0);


        double boxFlipInput = gamepad1.left_trigger + (gamepad1.left_bumper ? -1 : 0); //Servo has to be programmed as non CR
        double intakeInput = (gamepad1.dpad_down ? -1 : 0) + (gamepad1.dpad_up ? 1 : 0);    //Servo has to be programmed as CR

        double randomServoPower = (gamepad1.y ? 1 : 0) + (gamepad1.b ? -1 : 0);

        double grabberPower = gamepad1.a ? 1 : 0; //Servo has to be programmed as non CR
        double slidePower = (gamepad2.y ? 1 : 0) + (gamepad2.b ? -1 : 0);

        double intakePower = intakeInput * modifier_gamepad1Speed;
        double boxFlipPower = boxFlipInput * modifier_gamepad1Speed;

        // TODO Make leftPower + rightPower = 1 for max values of leftPower and rightPower
        double leftPower = leftInput * modifier_gamepad2Speed;
        double rightPower = rightInput * modifier_gamepad2Speed;


        // Send calculated power to wheels
        leftFrontMotor.setPower(leftPower);
        leftBackMotor.setPower(leftPower);
        rightFrontMotor.setPower(rightPower);
        rightBackMotor.setPower(rightPower);

        // Set slide power
       // slideMotor.setPower(slidePower);
        // Set boxFlip power
       // boxFlipServo.setPower(boxFlipPower);
        // Set intake power
        intakeServo.setPower(intakePower);
        // Set grabber power
        grabberServo.setPower(grabberPower);
        //set Random servo power
        randomServo.setPower(randomServoPower);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + TeleOP_runtime.toString());
        telemetry.addData("Motors", "Left = (%.2f), Right = (%.2f) Slide = (%.2f) ", leftPower, rightPower,slidePower);
        telemetry.addData("Servos", "boxFlip = (%.2f) randomServo = (%.2f) ", boxFlipPower, randomServoPower);
        telemetry.addData("Other servos", "Intake = (%.2f) grabber = (%.2f)", intakePower, grabberPower);
        telemetry.update();
    }

    //Run the code once 'Driver' hits STOP.
    @Override
    public void stop() {

    }

}
