package loki;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//imports


public abstract class Auto_Abstract extends LinearOpMode {
    //variables
    DcMotor lf, rf, lb, rb, ls;
    public Gamepad g1, g2;
    Servo clawL, clawR, hook;
    ColorSensor colorSensor, skyStoneColor;    // Hardware Device Object
    boolean bLedOn = true;

    private ElapsedTime runtime = new ElapsedTime();
    //Directions
    static final int FORWARD = 0;
    static final int BACKWARDS = 1;
    static final int STRAFE_RIGHT = 2;
    static final int STRAFE_LEFT = 3;
    static final int FREEFORM = 4;
    static final int UP = 0;
    static final int DOWN = 1;
    static final int CLOSE = 0;
    static final int OPEN = 1;
    static final int PART = 2;

    static final int RED = 0;
    static final int GREEN = 1;
    static final int BLUE = 2;
    static final int LUM = 3;




    //Encoder Setup
    static final double PI = Math.PI;
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);
    //initStuff (claw, hook, wheels)

    //other functions
    // todo: write your code here


    public void generalDefine(){
        //Motor Define
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        ls = hardwareMap.dcMotor.get("ls");
        //Servo Define
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");

        hook = hardwareMap.servo.get("hook");

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        skyStoneColor = hardwareMap.get(ColorSensor.class, "SkyColor");

        colorSensor.enableLed(bLedOn);
        skyStoneColor.enableLed(bLedOn);


        //color_sensor = hardwareMap.colorSensor.get("color");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);


        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    }
    public void drive(double power, double distance, int direction) { //parameters: When you use the function, the code will ask for these two variables

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);







        double sPower = 0.4;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        switch(direction){
            case FORWARD:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * -distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * -distance)); //For some reason, going foward gives negative encoder values
                lb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));

                lfPower = power;
                rfPower = power;
                lbPower = power;
                rbPower = power;
                break;


            case BACKWARDS:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                lfPower = -power;
                rfPower = -power;
                lbPower = -power;
                rbPower = -power;
                break;
            case STRAFE_RIGHT:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * -distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));

                lfPower = power;
                rfPower = -power;
                lbPower = -power;
                rbPower = power;
                break;
            case STRAFE_LEFT:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * -distance));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * distance));

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
            telemetry.addData("rb",rb.getCurrentPosition());
            telemetry.addData("rf",rf.getCurrentPosition());
            telemetry.addData("lf",lf.getCurrentPosition());
            telemetry.addData("lb",lb.getCurrentPosition());
            telemetry.update();
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

    public void monoColorDrive(double power, double color, int direction, int target) { //parameters: When you use the function, the code will ask for these two variables

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);







        double sPower = 0.4;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        switch(direction){
            case FORWARD:


                lfPower = power;
                rfPower = power;
                lbPower = power;
                rbPower = power;
                break;


            case BACKWARDS:

                lfPower = -power;
                rfPower = -power;
                lbPower = -power;
                rbPower = -power;
                break;
            case STRAFE_RIGHT:

                lfPower = power;
                rfPower = -power;
                lbPower = -power;
                rbPower = power;
                break;
            case STRAFE_LEFT:

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


        switch (target) {
            case RED:
                while (opModeIsActive() && (colorSensor.red() < color)) {
                    idle();
                    telemetry.addData("Red  ", colorSensor.red());
                }
                break;
            case GREEN:
                while (opModeIsActive() && (colorSensor.green() < color)) {
                    idle();
                    telemetry.addData("Green", colorSensor.green());
                }
                break;
            case BLUE:
                while (opModeIsActive() && (colorSensor.blue() < color)) {
                    idle();
                    telemetry.addData("Blue ", colorSensor.blue());
                }
                break;
            case LUM:
                while (opModeIsActive() && (colorSensor.alpha() < color)) {
                    idle();
                    telemetry.addData("Clear", colorSensor.alpha());
                }

        }

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);


    }

    public void monoColorDriveSky(double power, double color, int direction, int target) { //parameters: When you use the function, the code will ask for these two variables

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);







        double sPower = 0.4;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        switch(direction){
            case FORWARD:


                lfPower = power;
                rfPower = power;
                lbPower = power;
                rbPower = power;
                break;


            case BACKWARDS:

                lfPower = -power;
                rfPower = -power;
                lbPower = -power;
                rbPower = -power;
                break;
            case STRAFE_RIGHT:

                lfPower = power;
                rfPower = -power;
                lbPower = -power;
                rbPower = power;
                break;
            case STRAFE_LEFT:

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


        switch (target) {
            case RED:
                while (opModeIsActive() && (skyStoneColor.red() >= color)) {
                    idle();
                    telemetry.addData("Red  ", skyStoneColor.red());
                }
                break;
            case GREEN:
                while (opModeIsActive() && (skyStoneColor.green() >= color)) {
                    idle();
                    telemetry.addData("Green", skyStoneColor.green());
                }
                break;
            case BLUE:
                while (opModeIsActive() && (skyStoneColor.blue() >= color)) {
                    idle();
                    telemetry.addData("Blue ", skyStoneColor.blue());
                }
                break;
            case LUM:
                while (opModeIsActive() && (skyStoneColor.alpha() >= color)) {
                    idle();
                    telemetry.addData("Clear", skyStoneColor.alpha());
                }

        }

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);


    }

    public void hook(int direction){
        switch (direction) {
            case UP:
                hook.setPosition(.9);
                sleep(1200);
                break;
            case DOWN:
                hook.setPosition(0.15);
                sleep(1200);
                break;
        }
    }

    public void claw(int state){
        switch (state){
            case CLOSE:
                clawR.setPosition(.7); // close claw
                clawL.setPosition(.25);
                sleep(1000);
                break;
            case OPEN:
                clawR.setPosition(.2); //open claw
                clawL.setPosition(.75);
                sleep(1000);
                break;
            case PART:
                clawR.setPosition(.4); //partial claw
                clawL.setPosition(.6);
                sleep(1000);
        }
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



}