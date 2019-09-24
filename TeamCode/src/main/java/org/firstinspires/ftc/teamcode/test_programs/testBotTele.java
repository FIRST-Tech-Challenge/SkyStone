//Run from the necessary package
package org.firstinspires.ftc.teamcode.test_programs;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name="test bot") //Name the class
public class testBotTele extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    Servo glyphFlip;

    //Define relic motors
    Servo relicGrab;
    CRServo relicFlip;
    DcMotor relicSpool;

    //Define floats to be used as joystick inputs and trigger inputs
    float drivePower;
    float shiftPower;
    float leftTurnPower;
    float rightTurnPower;

    //Define an int to use as gamepad2 initialization
    int gamepad2Init = 0;

    //Define an elapsed time variable
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime flipTime = new ElapsedTime();
    private ElapsedTime relicTime = new ElapsedTime();

    //Define booleans to make relic movements and shut off the intake wheels when gamepad2 is initialized
    boolean bMoved = false;

    //Define a function to use to set motor powers
    public void setDriveMotorPowers(float leftFrontPower, float leftBackPower, float rightFrontPower, float rightBackPower)
    {
        //Use the entered powers and feed them to the motors
        leftMotorFront.setPower(leftFrontPower);
        leftMotorBack.setPower(leftBackPower);
        rightMotorFront.setPower(rightFrontPower);
        rightMotorBack.setPower(rightBackPower);
    }

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Get references to the DC Motors from the hardware map
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        //Get references to the Servo Motors from the hardware map
        glyphFlip = hardwareMap.servo.get("glyphFlip");
        relicGrab = hardwareMap.servo.get("relicGrab");
        relicFlip = hardwareMap.crservo.get("relicFlip");

        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set the drive motors to brake mode to prevent rolling due to chain
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Wait for start button to be clicked
        waitForStart();

        //Reset the flipper
        glyphFlip.setPosition(0.95);

        //Reset the runtime after the start button is clicked
        runtime.reset();

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {

            //DRIVE MOTOR CONTROLS
            //Set float variables as the inputs from the joysticks and the triggers
            drivePower = (float) -((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
            shiftPower = (float) -((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
            leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.75);
            rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.75);

            //Gamepad2 init
            if ((Math.abs(gamepad2.left_stick_y) > 0.1) && (gamepad2Init == 0))
                bMoved = true;

            //If gamepad2 is used, flip down the relic grabber and open the claws
            if (bMoved)
            {
                relicFlip.setPower(0.7);
                relicTime.reset();
                relicGrab.setPosition(0.32);
                bMoved = false;
                gamepad2Init++;

            }

            //Drive if the joystick is pushed more Y than X
            if (Math.abs(drivePower) > Math.abs(shiftPower))
                setDriveMotorPowers(-drivePower, drivePower, drivePower, -drivePower);

            //Shift if the joystick is pushed more on X than Y
            if (Math.abs(shiftPower) > Math.abs(drivePower))
                setDriveMotorPowers(shiftPower, shiftPower, shiftPower, shiftPower);

            //If the left trigger is pushed, turn left at that power
            if (leftTurnPower > 0)
            {
                setDriveMotorPowers(leftTurnPower, leftTurnPower, -leftTurnPower, -leftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurnPower > 0)
            {
                setDriveMotorPowers(-rightTurnPower, -rightTurnPower, rightTurnPower, rightTurnPower);
            }

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
            {
                setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
            }

            if (drivePower < -0.6)
            {
                glyphFlip.setPosition(0.75);
            }

            //If the dpad is pushed down, flip the glyphs into the cryptobox
            //Then reset the flipper
            if (gamepad1.dpad_down)
            {
                glyphFlip.setPosition(0.3);
                flipTime.reset();
                if (flipTime.time() > 1.2)
                {
                    glyphFlip.setPosition(0.75);
                }
            }

            if (relicTime.time() > 1.1)
            {
                relicFlip.setPower(0.0);
            }

            //If the x button is pressed, open the claws
            if (gamepad2.x)
            {
                relicGrab.setPosition(0.32);
            }

            //If the b button is pressed, close the
            if (gamepad2.b)
            {
                relicGrab.setPosition(1.00);
            }

            //If the y button is pressed, flip the relic flipper down
            //Also open the claws after a certain time to drop the relic
            if (gamepad2.y)
            {
                relicFlip.setPower(0.7);
                relicTime.reset();
//                functions.crServoTime(relicFlip, (float) 0.7, 1500);
            }

            //If the a button is pressed, flip the relic flipper up
            if (gamepad2.a)
            {
                //Up while holding relic, since it requires more time
//                functions.crServoTime(relicFlip, (float) -0.7, 3000);
                relicFlip.setPower(-0.7);
                relicTime.reset();
            }

            //Count time
            //Update the data
            telemetry.addData("Status", runtime);
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
