package org.firstinspires.ftc.teamcode.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {

    private static final double MAXLEVEL = 14;
    private static final double MAXHEIGHT = 35; // Inches
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
    static final double HORIZONTALEXTENSIONTIME = 5900; // Time it takes for lift to extend out = length of lift / speed of motors
    static final double INITIAL_HORIZONTALEXTENSIONTIME = 5500;
    static final double encoderLevelCount = (288 / (Math.PI * .53));

    static double LIFTPOWER = 1;
    static double HOOKDOWN = .60;
    static double HOOKUP = 1.0;


    double left_stick_y;
    double right_stick_y;

    double k = 1.0;
    double level = 0;
    double blockCount = 1.0;
    double blockHeight = 5.0; //Block Height In Inches
    double prevEncoderPos = 0;
    private boolean toggled = false;

    public void initOuttake(OpMode opMode) {

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
        if (prevEncoderPos > 0) {
            bottom = false;
        }

        resetLiftEncoders();



    }

    public void initOuttakeAuto(LinearOpMode opMode) {

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


        hookLeft.setDirection(Servo.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        prevEncoderPos = averageLiftPosition();
        if (prevEncoderPos > 0) {
            bottom = false;
        }

        resetLiftEncoders();

    }

    public void outTake_TeleOp()
    {

        horizontalLiftTele();
        raiseLiftMacro();
        hookToggle();
        openBasket();
        lift();
        encoderCalibrate();



        if(blockCount == 2)
        {
            level++;
            blockCount = 0;
        }

        if(Math.abs(opMode.gamepad2.left_trigger) > .5)
        {
            pushBlock.setPosition(.6);
        }
        else if(Math.abs(opMode.gamepad2.right_trigger) > .5)
        {
            pushBlock.setPosition(1);
        }

        if(opMode.gamepad2.right_bumper && opMode.gamepad2.left_bumper)
        {
            initHorizontalExtension();
        }

    }


    public void outTake_Auto(LinearOpMode opMode) {
       // initHorizontalExtension();
        pushBlock.setPosition(1);
        raiseLiftAuto();
        openBasketAuto();
        resetOuttake();

        if(blockCount == 2)
        {
            level++;
            blockCount = 0;
        }



    }

    public int getNumberOfBlocks()
    {
        return (int) (level * 2 + blockCount);
    }

    private void resetLiftEncoders() {
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // once coordinates placed into method robot auto aligns the whole output system to
    // drop the block off on at the right spot

    // Process : Robot Aligns, Lift Extends Up, CRServos move forward while hook connected to servo
    // is in contact with block, when block needs to be released hook from servo extended out

    public void hookAuto(LinearOpMode opmode1) {
        hookRight.setPosition(HOOKDOWN);
        hookLeft.setPosition(HOOKDOWN);

        drive.encoderMove(opMode1, DISTANCE_TO_BUILD_ZONE, 5, 1);

        //possible add a test case to make sure robot has foundation

        hookLeft.setPosition(HOOKUP);
        hookRight.setPosition(HOOKUP);
    }

    public double averageLiftPosition() {
        int count = 2;

        if (liftRight.getCurrentPosition() == 0 && !bottom) count--;
        if (liftLeft.getCurrentPosition() == 0 && !bottom) count--;
        if (count == 0) return 0;
        return (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / count;

    }

    public void raiseLiftMacro() {
        if (opMode.gamepad2.dpad_up) {
            liftRight.setPower(LIFTPOWER);
            liftLeft.setPower(LIFTPOWER);

            while (encoderLevelCount * blockHeight * level + .5 * encoderLevelCount > averageLiftPosition()) { }

            liftLeft.setPower(0);
            liftRight.setPower(0);
            top = true;
        }

    }

    public void initHorizontalExtension()
    {
        rightVex.setPower(.5);
        leftVex.setPower(-.5);

        time.reset();
        while(time.milliseconds() < INITIAL_HORIZONTALEXTENSIONTIME)
        {
            if(opMode.gamepad2.right_stick_button)
            {
                rightVex.setPower(0);
                leftVex.setPower(0);
            }
        }

        rightVex.setPower(0);
        leftVex.setPower(0);

    }

    private void raiseLiftAuto() {
        liftRight.setPower(LIFTPOWER);
        liftLeft.setPower(LIFTPOWER);

        while (encoderLevelCount * blockHeight * level + 3 * encoderLevelCount > averageLiftPosition()) {

            if(top && averageLiftPosition() > MAXHEIGHT * encoderLevelCount)
            {
                liftLeft.setPower(0);
                liftRight.setPower(0);
                return;
            }
        }

        liftLeft.setPower(0);
        liftRight.setPower(0);
        top = true;
    }

    //moves lift up and down by increments

    public void lift() {

        left_stick_y = opMode.gamepad2.left_stick_y;


        if (Math.abs(left_stick_y) > .05) {

            liftRight.setPower(-left_stick_y);
            liftLeft.setPower(-left_stick_y);

            top = false;
            bottom = false;
        }
        else {
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }
        if (averageLiftPosition() <= 0) {
            bottom = true;
            resetLiftEncoders();
        }
        else if (averageLiftPosition() >= MAXHEIGHT * encoderLevelCount) {
            top = true;
        }

        if (top && -left_stick_y > 0) {
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }
        else if (bottom && -left_stick_y < 0) {

            liftRight.setPower(0);
            liftLeft.setPower(0);
        }


        if (opMode.gamepad2.b && !bottom && averageLiftPosition() > 4 * encoderLevelCount)
        {
            time.reset();
            while (time.milliseconds() < 100) { }
            resetOuttake();
        }
    }

    public void encoderCalibrate()
    {
        if(opMode.gamepad2.left_stick_button && opMode.gamepad2.right_stick_button)
        {
            time.reset();
            while(time.milliseconds() < 100){}
            resetLiftEncoders();
        }
    }


    public void Output_Telemetry()
    {
        opMode.telemetry.addData("" , pushBlock.getPosition());
        opMode.telemetry.addData("Lift Bottom : ", bottom);
        opMode.telemetry.addData("Lift Top : ", top);
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


    public void openBasketAuto()
    {
        blockCount++;

        pushBlock.setPosition(1);

        rightVex.setPower(.5);
        leftVex.setPower(-.5);

        //8.78 inches extends out
        time.reset();
        while(time.milliseconds() < HORIZONTALEXTENSIONTIME)
        {
        }
        //set position direction on angle - ask  trevor

        rightVex.setPower(0);
        leftVex.setPower(0);
    }
    //  opens up the output basket using the Servos
    public void openBasket()
    {
        // pushes front servo in while as rotating CRServos to open basket
        if(opMode.gamepad2.a) {
            blockCount++;

            pushBlock.setPosition(1);

            rightVex.setPower(.5);
            leftVex.setPower(-.5);

            //8.78 inches extends out
            time.reset();
            while(time.milliseconds() < HORIZONTALEXTENSIONTIME)
            {
                if(opMode.gamepad2.right_stick_button)
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
    }

    //  resets all variables and sets lift back to initial position
    public void resetOuttake()
    {

        if(hookRight.getPosition() != 1)
        {
            hookRight.setPosition(1);
            hookLeft.setPosition(1);
        }


        if(bottom && averageLiftPosition() < 4 * encoderLevelCount)
        {
           return;
        }

        opMode.telemetry.addData("RESET OUTTAKE IS RUNNING", " PREPARE LEFT BUTTON / RIGHT BUTTON");
        opMode.telemetry.update();


        pushBlock.setPosition(1);

        rightVex.setPower(-.5);
        leftVex.setPower(.5);

        time.reset();
        while(time.milliseconds() < HORIZONTALEXTENSIONTIME) {

            if(opMode.gamepad2.right_stick_button)
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
        while(averageLiftPosition() >= 4 * encoderLevelCount && time.milliseconds() < 1000) {

            if(opMode.gamepad2.right_stick_button)
            {
                liftLeft.setPower(0);
                liftRight.setPower(0);
            }

        }

        liftLeft.setPower(0);
        liftRight.setPower(0);

        top = false;
        bottom = true;


        pushBlock.setPosition(.6);

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

        right_stick_y = opMode.gamepad2.right_stick_y;

        if (Math.abs(right_stick_y) >= .075) {
            rightVex.setPower(-right_stick_y / 2);
            leftVex.setPower(right_stick_y / 2);
        }
        else {
            rightVex.setPower(0);
            leftVex.setPower(0);

        }
    }
}
