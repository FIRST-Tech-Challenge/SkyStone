package loki;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Auto_Foundation_Red")
public class Auto_Foundation_Red extends Auto_Abstract{
    DcMotor lf, rf, lb, rb, ls;
    public Gamepad g1, g2;
    Servo clawL, clawR, hook;
    private ElapsedTime runtime = new ElapsedTime();
    //Directions
    static final int FORWARD = 0;
    static final int BACKWARDS = 1;
    static final int STRAFE_RIGHT = 2;
    static final int STRAFE_LEFT = 3;
    static final int FREEFORM = 4;
    static final int UP = 0;
    static final int DOWN = 1;


    //Encoder Setup
    static final double PI = Math.PI;
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);



    @Override
    public void runOpMode(){


        generalDefine();
        //----------------------------------------------------------------------------------------------------------------//
        //Foundation Only
        //Move 120 Inches
        waitForStart();
        telemetry.addData("Waiting", "...");
        telemetry.update();
        //sleep(5000);
        claw(OPEN);
        hook(UP);
        drive(0.4,30, BACKWARDS);
        hook(DOWN);
        //sleep(3000);
        drive(0.4, 29.7,FORWARD);
        //sleep(3000);
        hook(UP);
        monoColorDrive(0.3,5,STRAFE_RIGHT, RED);
        //drive(0.3,61,STRAFE_RIGHT);
        //sleep(3000);

        //Skystone & Foundation Theory
        /*Create 2 autonomouses: 1 for left skystone and one for right skystone
         * Need Color Sensonr
         * Left skystone: Start from left and scan until hit block that isn't yellow
         * Right skystone: Start from right and scan until hit block that isn't yellow
         *
         * Take one skystone to foundation, let other team take other one
         * One team does foundation, one does not
         * Park
         * */








        // Step 1: Move Forward 30.25 Inches

        // Step 2: Grab The Foundation
        // Step 3: Pull The Foundation Back In To The Depot 29 Inches
        // Step 4: Strafe Left 31 Inches
        //
    }

    //Tiles are 24x24 inches

}
