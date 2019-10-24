package org.firstinspires.ftc.teamcode.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake{

    private static final double MAXLEVEL = 14;
    private static final double DISTANCE_TO_BUILD_ZONE = 1; // what ever distance is from foundation to build zone
    public Servo pushBlock;
    public Servo hookRight;
    public Servo hookLeft;
    public CRServo rightVex;
    public CRServo leftVex;
    public DcMotor liftRight;
    public DcMotor liftLeft;

    LinearOpMode opMode;
    ElapsedTime time = new ElapsedTime();

    //1.055 Inches Base, Foundation is 2.25 inches

    boolean top;
    boolean bottom;
    boolean blockInLift;

    static final double distanceBetweenBlocks = 4.0; // In Inches
    static final double liftExtensionTime = 1000; // Time it takes for lift to extend out = length of lift / speed of motors

    static final double encoderLevelCount = (1440 / Math.PI) ;

    static double LIFTPOWER = 1.0;
    static double HOOKDOWN = -1.0;
    static double HOOKUP = 1.0;

    double k = 1.0;
    double level = 1.0;
    double blockCount = 1.0;
    double blockHeight = 5.0; //Block Height In Inches
    private boolean toggled = false;

    public void initOuttake(OpMode opMode)
    {

        time.reset();

        top = false;
        bottom = false;
        blockInLift = false;

        k = 1.0;
        level = 1.0;
        blockCount = 0.0;


        pushBlock = opMode.hardwareMap.servo.get("PB");
        rightVex = opMode.hardwareMap.crservo.get("ROut");
        leftVex = opMode.hardwareMap.crservo.get("LOut");
        liftLeft = opMode.hardwareMap.dcMotor.get("LLift");
        liftRight = opMode.hardwareMap.dcMotor.get("RLift");
        hookLeft = opMode.hardwareMap.servo.get("LHook");
        hookRight = opMode.hardwareMap.servo.get("RHook");

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opMode.telemetry.addData("Success", "Outtake Initialized");
        opMode.telemetry.update();


        resetOuttake();
    }


    // once coordinates placed into method robot auto aligns the whole output system to
    // drop the block off on at the right spot

    // Process : Robot Aligns, Lift Extends Up, CRServos move forward while hook connected to servo
    // is in contact with block, when block needs to be released hook from servo extended out

    public void hookAuto(DriveTrain drive)
    {
        hookRight.setPosition(HOOKDOWN);
        hookLeft.setPosition(HOOKDOWN);

        //assuming robot is lined up with foundation
        raiseLift();
        //openBasket();

        drive.encoderMove(opMode, DISTANCE_TO_BUILD_ZONE, 5, 270);

        //possible add a test case to make sure robot has foundation

        hookLeft.setPosition(HOOKUP);
        hookRight.setPosition(HOOKUP);
    }
    public double averageLiftPosition()
    {
        int count = 2;

        if(liftRight.getCurrentPosition() == 0) count--;
        if(liftLeft.getCurrentPosition() == 0) count--;
        if(count == 0) return 0;
        return (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / count;

    }

    public void raiseLift()
    {
        liftRight.setPower(LIFTPOWER);
        liftLeft.setPower(LIFTPOWER);

        while(encoderLevelCount * blockHeight * level + 320  > averageLiftPosition())
        {
        }


        liftLeft.setPower(0);
        liftRight.setPower(0);
    }

    public void outTake_Auto(DriveTrain drive)
    {

        raiseLift();

        if(blockCount % 2 == 1)
        {
            //openBasket();
        }
        else if(blockCount % 2 == 0)
        {
            drive.strafeMove(opMode, 4, 3, 1);

            //openBasket();

            drive.strafeMove(opMode,  4, 3, -1);

            level++;
        }

        //resetOuttake();
    }

    //moves lift up and down by increments

    public void outTake_TeleOp(OpMode opMode)
    {
        if (Math.abs(opMode.gamepad2.left_stick_y) > .05) {
            liftRight.setPower(opMode.gamepad2.left_stick_y);
            liftLeft.setPower(opMode.gamepad2.left_stick_y);
        }
        else {
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }
        if(opMode.gamepad2.dpad_up && level < MAXLEVEL)
        {
            // move lift up

            liftRight.setPower(.1);
            liftLeft.setPower(.1);



            while(encoderLevelCount * blockHeight * level + 320  > liftLeft.getCurrentPosition())
            {
            }

            level += 1;

        }
        else if(opMode.gamepad2.dpad_down && level > 1)
        {
            // move lift down

            liftRight.setPower(-LIFTPOWER * k);
            liftLeft.setPower(-LIFTPOWER * k);

            level -= 1;

            while(encoderLevelCount * blockHeight * (level - 1) - 320 < liftLeft.getCurrentPosition())
            {
            }
        }
        else
        {
            // stop lift

            liftRight.setPower(0);
            liftLeft.setPower(0);
        }

        //  Extend Outtake out, and activate servo to push block forward

        if(opMode.gamepad2.a && pushBlock.getPosition() == .5)
        {
            openBasket();
        }
        else if(opMode.gamepad2.a && pushBlock.getPosition() != .5)
        {
            pushBlock.setPosition(.5);
        }
        else if(opMode.gamepad2.b)
        {
            resetOuttake();
        }

        // lift proportional changing, for precision placement


        if(opMode.gamepad2.dpad_left)
        {
            k -= .1;
        }
        else if(opMode.gamepad2.dpad_right)
        {
            k += .1;
        }

        hookToggle();

        opMode.telemetry.addData("Lift Power", k);
        opMode.telemetry.update();
    }

    //  opens up the output basket using the Servos
    public void openBasket()
    {
        // pushes front servo in while as rotating CRServos to open basket
        time.reset();

        blockCount++;

        rightVex.setPower(1);
        leftVex.setPower(1);

        //8.78 inches extends out
       // while((rightSideY.getCurrentPosition() + leftSideY) / 2 < 8.78 * 1440)
        //{
        //}

        rightVex.setPower(0);
        leftVex.setPower(0);

        pushBlock.setPosition(1); //set position direction on angle - ask  trevor

        // should push block out at that point

        pushBlock.setPosition(.5); // back to open position

    }

    //  resets all variables and sets lift back to initial position
    public void resetOuttake()
    {


        pushBlock.setPosition(.5); // moves servo to open position what ever angle that is

        rightVex.setPower(-1);
        leftVex.setPower(-1);
        time.reset();
        while(liftExtensionTime > time.milliseconds())
        {

        }

        rightVex.setPower(0);
        leftVex.setPower(0);

        time.reset();
        while(liftRight.getCurrentPosition() > 0 && liftLeft.getCurrentPosition() > 0 && time.milliseconds() < 5000) // once lift registers to bottom then bottom will equal to true
        {
            liftLeft.setPower(-LIFTPOWER);
            liftRight.setPower(-LIFTPOWER);
        }

        top = false;
        bottom = true;

        if(hookRight.getPosition() != 1)
        {
            hookRight.setPosition(1);
            hookLeft.setPosition(1);
        }

    }

    public void hookToggle()
    {
        if(!toggled && opMode.gamepad2.y)
        {
            toggled = true;

            //set hook position to what ever the angle is, I assume servo is at (1) = 180,
            // and moves to (0) = 0 degrees

            hookLeft.setPosition(HOOKDOWN);
            hookRight.setPosition(HOOKDOWN);
        }
        else if(toggled && opMode.gamepad2.y)
        {
            toggled = false;

            hookLeft.setPosition(HOOKUP);
            hookRight.setPosition(HOOKUP);
        }
    }

    public void horizontalLiftTele(OpMode opMode) {
        if (Math.abs(opMode.gamepad2.right_stick_y) > .05) {
            rightVex.setPower(opMode.gamepad2.right_stick_y);
            leftVex.setPower(opMode.gamepad2.right_stick_y);
        }
        else {
            rightVex.setPower(0);
            leftVex.setPower(0);
        }
    }
}
