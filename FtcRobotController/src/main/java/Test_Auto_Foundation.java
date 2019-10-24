
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Test_Foundation_Auto")
public class Test_Auto_Foundation extends LinearOpMode {
    DcMotor lf, rf, lb, rb;
    public Gamepad g1, g2;
    Servo clawL, clawR;
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
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double GEAR_DIAMETER_INCHES = 1.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (GEAR_DIAMETER_INCHES * PI);



    @Override
    public void runOpMode(){


        //Motor Define
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        //Servo Define
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        //Encoder Stuff
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //----------------------------------------------------------------------------------------------------------------//
        //Foundation Only
        //Move 120 Inches
        waitForStart();
        telemetry.addData("Waiting", "...");
        telemetry.update();
        //sleep(5000);
        drive(30.25, FORWARD);
        hook(DOWN);
        //sleep(3000);
        drive(29,BACKWARDS);
        //sleep(3000);
        hook(UP);
        drive(31,STRAFE_LEFT);
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
    public void drive(double distance, int direction) {

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






        double power = 0.8;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        switch(direction){
            case FORWARD:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * distance));

                lfPower = power;
                rfPower = power;
                lbPower = power;
                rbPower = power;
                break;
            case BACKWARDS:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * -distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * -distance));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));
                lfPower = -power;
                rfPower = -power;
                lbPower = -power;
                rbPower = -power;
                break;
            case STRAFE_RIGHT:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * -distance));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * distance));

                lfPower = power;
                rfPower = -power;
                lbPower = -power;
                rbPower = power;
                break;
            case STRAFE_LEFT:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * -distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));

                lfPower = -power;
                rfPower = power;
                lbPower = power;
                rbPower = -power;
                break;
        }



        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()&& (lf.isBusy() || lb.isBusy() || rb.isBusy() || rf.isBusy())) {
            idle();
        }
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void freeform(int distance, int angle){ //angle in radians
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setTargetPosition((int) (COUNTS_PER_INCH * distance)); //distance needs to be in inches
        rf.setTargetPosition((int) (COUNTS_PER_INCH * distance));
        lb.setTargetPosition((int) (COUNTS_PER_INCH * distance));
        rb.setTargetPosition((int) (COUNTS_PER_INCH * distance));



        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

         //Trig
        //double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //returns hypotonuse (C value in triangle)

        double robotAngle = angle - Math.PI / 4;
        //return angle x (next to center of circle)

        //double rightX = -gamepad1.right_stick_x;
        //rotiation

        final double lfPow =  Math.sin(angle);
        final double rfPow =  Math.cos(angle);
        final double lbPow =  Math.cos(angle);
        final double rbPow =  Math.sin(angle);
        //determines wheel power

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }





    public void hook(int direction){
        switch (direction) {
            case UP:
                clawR.setPosition(1);
                clawL.setPosition(0);
                break;
            case DOWN:
                clawR.setPosition(0);
                clawL.setPosition(1);
                break;

        }



    }

}