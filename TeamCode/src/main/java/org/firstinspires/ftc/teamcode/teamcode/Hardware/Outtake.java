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
    private static final double MAXHEIGHT = 30; // Inches
    private static final double DISTANCE_TO_BUILD_ZONE = 1; // what ever distance is from foundation to build zone
    public Servo pushBlock;
    public Servo hookRight;
    public Servo hookLeft;
    public CRServo rightVex;
    public CRServo leftVex;
    public DcMotor liftRight;
    public DcMotor liftLeft;

    OpMode opMode;
    LinearOpMode opMode1;
    DriveTrain drive = new DriveTrain();
    ElapsedTime time = new ElapsedTime();

    //1.055 Inches Base, Foundation is 2.25 inches

    boolean top;
    boolean bottom;

    static final double DISTANCE_BETWEEN_BLOCKS = 4.0; // In Inches
    static final double HORIZONTALEXTENSIONTIME = 2000; // Time it takes for lift to extend out = length of lift / speed of motors

    static final double encoderLevelCount = (288 /  (Math.PI * .53)) ;

    static double LIFTPOWER = 1;
    static double HOOKDOWN = .60;
    static double HOOKUP = 1.0;

    double k = 1.0;
    double level = 0;
    double blockCount = 1.0;
    double blockHeight = 5.0; //Block Height In Inches
    double prevEncoderPos = 0;
    private boolean toggled = false;

    public void initOuttake(OpMode opMode)
    {

        time.reset();
        this.opMode = opMode;
        top = false;
        bottom = true;

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


        hookLeft.setDirection(Servo.Direction.FORWARD);
        hookRight.setDirection(Servo.Direction.FORWARD);

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        prevEncoderPos = averageLiftPosition();
        if(prevEncoderPos > 0)
        {
            bottom = false;
        }

        //resetOuttake();
        resetLiftEncoders();


    }

    public void initOuttakeAuto(LinearOpMode opMode)
    {

        time.reset();
        this.opMode = opMode;
        top = false;
        bottom = false;

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


        hookLeft.setDirection(Servo.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        prevEncoderPos = averageLiftPosition();

        resetLiftEncoders();

    }
    private void resetLiftEncoders()
    {
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // once coordinates placed into method robot auto aligns the whole output system to
    // drop the block off on at the right spot

    // Process : Robot Aligns, Lift Extends Up, CRServos move forward while hook connected to servo
    // is in contact with block, when block needs to be released hook from servo extended out

    public void hookAuto(LinearOpMode opmode1)
    {
        hookRight.setPosition(HOOKDOWN);
        hookLeft.setPosition(HOOKDOWN);

        drive.encoderMove(opMode1, DISTANCE_TO_BUILD_ZONE, 5, 1);

        //possible add a test case to make sure robot has foundation

        hookLeft.setPosition(HOOKUP);
        hookRight.setPosition(HOOKUP);
    }
    public double averageLiftPosition()
    {
        int count = 2;

        if(liftRight.getCurrentPosition() == 0 && !bottom) count--;
        if(liftLeft.getCurrentPosition() == 0 && !bottom) count--;
        if(count == 0) return 0;
        return (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / count;

    }

    public void raiseLift()
    {
        resetLiftEncoders();
        liftRight.setPower(LIFTPOWER);
        liftLeft.setPower(LIFTPOWER);

        while(encoderLevelCount * blockHeight * level + 3 * encoderLevelCount  > averageLiftPosition())
        {
        }

        liftLeft.setPower(0);
        liftRight.setPower(0);
        top = true;
    }

    public void outTake_Auto(LinearOpMode opMode)
    {
        pushBlock.setPosition(1);
        raiseLift();
        openBasket();


    }

    //moves lift up and down by increments



    public void outTake_TeleOp()
    {

        horizontalLiftTele();

        if(blockCount == 2)
        {
            level++;
            blockCount = 0;
        }

        if(averageLiftPosition() <= 0)
        {
            bottom = true;
            resetLiftEncoders();
        }

        if(averageLiftPosition() >= MAXHEIGHT * encoderLevelCount)
        {
            top = true;
        }

        if(Math.abs(opMode.gamepad2.left_trigger) > .5)
        {
            pushBlock.setPosition(0);
        }
        else if(Math.abs(opMode.gamepad2.right_trigger) > .5)
        {
            pushBlock.setPosition(1);
        }

        if (Math.abs(opMode.gamepad2.left_stick_y) > .05) {

            liftRight.setPower(-opMode.gamepad2.left_stick_y * k);
            liftLeft.setPower(-opMode.gamepad2.left_stick_y * k);
        }
        else if(top && opMode.gamepad2.left_stick_y > 0)
        {
            top = false;
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }
        else if(bottom && opMode.gamepad2.left_stick_y < 0)
        {
            bottom = false;
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }
        else
        {
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }

        //  Extend Outtake out, and activate servo to push block forward
       if(opMode.gamepad2.a) {
            openBasket();
            //  Push block onto field

        }
        else if(!bottom && averageLiftPosition() > 0 && opMode.gamepad2.b) {

            resetOuttake();

        }
        // lift proportional changing, for precision placement


       /* if(opMode.gamepad2.dpad_left)
        {
            time.reset();
            while(time.milliseconds() < 100){}

            k -= 0.3;
        }
        else if(opMode.gamepad2.dpad_right)
        {
            time.reset();
            while(time.milliseconds() < 100){}
            k += 0.3;
        }*/

        if(opMode.gamepad2.dpad_up)
        {
            raiseLift();
        }

        hookToggle();

    }

    public void Output_Telemetry()
    {
        //opMode.telemetry.addData("K : ", k);
        opMode.telemetry.addData("Lift Bottom : ", bottom);
        opMode.telemetry.addData("Lift Right : ", liftRight.getCurrentPosition());
        opMode.telemetry.addData("Lift Left : ", liftLeft.getCurrentPosition());
        opMode.telemetry.addData("Prev Encoder Pos : ", prevEncoderPos);
        opMode.telemetry.addData("Average : ", averageLiftPosition());
        opMode.telemetry.addData("Block Count", blockCount);
        opMode.telemetry.addData("Level", level);
        opMode.telemetry.addData("Vex Power Right", rightVex.getPower());
        opMode.telemetry.addData("Vex Power Left", leftVex.getPower());
        opMode.telemetry.addData("Left Hook", hookLeft.getPosition());
        opMode.telemetry.addData("Right Hook", hookRight.getPosition());
    }

    //  opens up the output basket using the Servos
    public void openBasket()
    {
        // pushes front servo in while as rotating CRServos to open basket


        blockCount++;

        pushBlock.setPosition(1);

        rightVex.setPower(-.5);
        leftVex.setPower(.5);

        //8.78 inches extends out
        time.reset();
        while(time.milliseconds() < HORIZONTALEXTENSIONTIME)
        {
            if(Math.abs(opMode.gamepad2.right_stick_x) > 0)
            {
                rightVex.setPower(0);
                leftVex.setPower(0);
            }
        }
        //set position direction on angle - ask  trevor

        rightVex.setPower(0);
        leftVex.setPower(0);



        // should push block out at that point

        // back to open position

    }

    //  resets all variables and sets lift back to initial position
    public void resetOuttake()
    {

/*
        if(hookRight.getPosition() != 1)
        {
            hookRight.setPosition(1);
            hookLeft.setPosition(1);
        }
*/

        if(!bottom && averageLiftPosition() > 0)
        {
            return;
        }

        pushBlock.setPosition(1); // moves servo to open position what ever angle that is

        rightVex.setPower(.5);
        leftVex.setPower(-.5);

        time.reset();
        while(time.milliseconds() < HORIZONTALEXTENSIONTIME) {

            if(Math.abs(opMode.gamepad2.right_stick_x) > 0)
            {
                rightVex.setPower(0);
                leftVex.setPower(0);
            }
        }

        rightVex.setPower(0);
        leftVex.setPower(0);



        liftLeft.setPower(-LIFTPOWER);
        liftRight.setPower(-LIFTPOWER);

        time.reset();
        while(averageLiftPosition() >= 10 && time.milliseconds() < 3000) {

            if(Math.abs(opMode.gamepad2.right_stick_x) > 0)
            {
                liftLeft.setPower(0);
                liftRight.setPower(0);
            }

        }

        liftLeft.setPower(0);
        liftRight.setPower(0);

        top = false;
        bottom = true;


        resetLiftEncoders();

        pushBlock.setPosition(0);
    }

    public void hookToggle()
    {
        if(!toggled && opMode.gamepad2.y)
        {
            toggled = true;

            time.reset();
            while(time.milliseconds() < 100)
            {

            }
            //set hook position to what ever the angle is, I assume servo is at (1) = 180,
            // and moves to (0) = 0 degrees

            hookLeft.setPosition(0);
            hookRight.setPosition(1);
        }
        else if(toggled && opMode.gamepad2.y)
        {
            toggled = false;


            time.reset();
            while (time.milliseconds() < 100)
            {

            }
            hookLeft.setPosition(1);
            hookRight.setPosition(0);
        }
    }

    public void horizontalLiftTele() {
        if (Math.abs(opMode.gamepad2.right_stick_y) >= .075) {
            rightVex.setPower(opMode.gamepad2.right_stick_y / 2);
            leftVex.setPower(-opMode.gamepad2.right_stick_y / 2);
        }
        else {
            rightVex.setPower(0);
            leftVex.setPower(0);

        }
    }
}
