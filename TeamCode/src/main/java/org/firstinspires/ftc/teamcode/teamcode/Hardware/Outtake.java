package org.firstinspires.ftc.teamcode.teamcode.Hardware;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake{

    private static final double MAXLEVEL = 14;
    private static final double MAXHEIGHT = 47; // Inches
    private static final double DISTANCE_TO_BUILD_ZONE = 1; // what ever distance is from foundation to build zone
    public Servo pushBlock;
    public Servo hookRight;
    public Servo hookLeft;
    public CRServo rightVex;
    public CRServo leftVex;
    public DcMotor liftRight;
    public DcMotor liftLeft;

    OpMode opMode;
    ElapsedTime time = new ElapsedTime();

    //1.055 Inches Base, Foundation is 2.25 inches

    boolean top;
    boolean bottom;
    boolean blockInLift;
    boolean liftReset = false;

    static final double DISTANCE_BETWEEN_BLOCKS = 4.0; // In Inches
    static final double liftExtensionTime = 1000; // Time it takes for lift to extend out = length of lift / speed of motors

    static final double encoderLevelCount = (288 /  (Math.PI * .53)) ;

    static double LIFTPOWER = 1.0;
    static double HOOKDOWN = 0;
    static double HOOKUP = 1.0;

    double k = 1.0;
    double level = 1.0;
    double blockCount = 1.0;
    double blockHeight = 5.0; //Block Height In Inches
    private boolean toggled = false;


    public void initOuttake(OpMode opMode)
    {
        time.reset();
        this.opMode = opMode;
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


        hookLeft.setDirection(Servo.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opMode.telemetry.addData("Success", "Outtake Initialized");
        opMode.telemetry.update();

    }
    public void resetLiftEncoders()
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

    public void hookAuto(DriveTrain drive)
    {
        hookRight.setPosition(HOOKDOWN);
        hookLeft.setPosition(HOOKDOWN);

        drive.encoderMove((LinearOpMode)opMode, DISTANCE_TO_BUILD_ZONE, 5, 1);

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
        resetLiftEncoders();
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
            openBasket();
        }
        else if(blockCount % 2 == 0)
        {
            drive.strafeMove((LinearOpMode)opMode, DISTANCE_BETWEEN_BLOCKS, 3, 1);

            openBasket();

            drive.strafeMove((LinearOpMode)opMode,  DISTANCE_BETWEEN_BLOCKS, 3, -1);

            level++;
        }

        resetOuttake();
    }

    //moves lift up and down by increments

    public void outTake_TeleOp()
    {

        horizontalLiftTele();
        resetLiftEncoders();
        if(Math.abs(opMode.gamepad2.left_trigger) > .5)
        {
            pushBlock.setPosition(1);
        }
        else if(Math.abs(opMode.gamepad2.right_trigger) > .5)
        {
            pushBlock.setPosition(0);
        }

        if (Math.abs(opMode.gamepad2.left_stick_y) > .05 && averageLiftPosition() < MAXHEIGHT * encoderLevelCount) {
            liftRight.setPower(opMode.gamepad2.left_stick_y);
            liftLeft.setPower(opMode.gamepad2.left_stick_y);
        }
        else if(Math.abs(opMode.gamepad2.left_stick_y) > .05)
        {
            liftRight.setPower(-.5);
            liftLeft.setPower(-.5);
        }

        //  Extend Outtake out, and activate servo to push block forward
        else if(opMode.gamepad2.a && pushBlock.getPosition() == 0) {
            //  openBasket();
            //  Push block onto field

        }
        else if(opMode.gamepad2.a && pushBlock.getPosition() != 0) {

            //  Reset Push block hook to normal position
            //  pushBlock.setPosition(.5);
        }
        else if(opMode.gamepad2.b) {
            while (opMode.gamepad2.b){};
            resetOuttake();
        }
        else {
            liftRight.setPower(0);
            liftLeft.setPower(0);
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

    }

    //  opens up the output basket using the Servos
    public void openBasket()
    {
        // pushes front servo in while as rotating CRServos to open basket


        blockCount++;

        rightVex.setPower(1);
        leftVex.setPower(-1);

        //8.78 inches extends out
        time.reset();
        while(time.milliseconds() < 1000)
        {
        }

        rightVex.setPower(0);
        leftVex.setPower(0);

        pushBlock.setPosition(1); //set position direction on angle - ask  trevor

        // should push block out at that point

        pushBlock.setPosition(.5); // back to open position

    }

    //  resets all variables and sets lift back to initial position
    public void resetOuttake()
    {

        resetLiftEncoders();
        if(liftReset) return;

        pushBlock.setPosition(0); // moves servo to open position what ever angle that is

        rightVex.setPower(-1);
        leftVex.setPower(1);
        time.reset();
        while(2000 > time.milliseconds())
        {

        }

        rightVex.setPower(0);
        leftVex.setPower(0);

        time.reset();

        liftLeft.setPower(-LIFTPOWER);
        liftRight.setPower(-LIFTPOWER);

        while(averageLiftPosition() > 1 * encoderLevelCount && time.milliseconds() < 2000) // once lift registers to bottom then bottom will equal to true
        {
        }
        opMode.telemetry.addData("Lift : ", averageLiftPosition());
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
            while(opMode.gamepad2.y){};
            toggled = true;

            //set hook position to what ever the angle is, I assume servo is at (1) = 180,
            // and moves to (0) = 0 degrees

            hookLeft.setPosition(HOOKDOWN);
            hookRight.setPosition(HOOKDOWN);
        }
        else if(toggled && opMode.gamepad2.y)
        {
            while(opMode.gamepad2.y){};
            toggled = false;

            hookLeft.setPosition(HOOKUP);
            hookRight.setPosition(HOOKUP);
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
