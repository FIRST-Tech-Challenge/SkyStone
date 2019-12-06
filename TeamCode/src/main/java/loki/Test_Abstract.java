package loki;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public abstract class Test_Abstract extends OpMode {

    public DcMotor lf, rf, lb, rb, ls; //Define Motors In Code
    public Gamepad g1, g2;
    public Servo clawL, clawR, hook;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime hookTime = new ElapsedTime();
    private ElapsedTime slideTime = new ElapsedTime();
    private ElapsedTime clawTime = new ElapsedTime();
    ColorSensor color_sensor;
    float hsvValues[] = {0F, 0F, 0F};
    boolean up = true;
    boolean hClosed = false;
    BNO055IMU imu;
    double startOrient;
    int lMod = 0;
    int cPos = 0;


    public void initStuff() {
        telemetry.addData("Status", "Initialized");


        //Motor Define In Phone
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        ls = hardwareMap.dcMotor.get("ls");
        //Servo Define
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");

        hook = hardwareMap.servo.get("hook");

        //Gyro Stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //color_sensor = hardwareMap.colorSensor.get("color");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Runs based on speed instead of voltage; makes run more consistently
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void initColor(){
        //color_sensor = hardwareMap.colorSensor.get("color");
        color_sensor = hardwareMap.get(ColorSensor.class, "sensor_color");


    }

    private double getHeading()
    {
        //get's current angle/way you are facing
        final Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;

    }

    public void fieldOrient() {
        final double x = gamepad1.left_stick_x;
        final double y = gamepad1.left_stick_y;
        double speed = Math.sqrt(x * x + y * y);
        //returns hypotonuse (C value in triangle)

        //calculate difference in starting angle and current angle
        final double currentHeading = getHeading();
        double angleDiff = startOrient - currentHeading;

        double robotAngle = Math.atan2(y, x) - angleDiff;
        //return angle x (next to center of circle)

        double rightX = gamepad1.right_stick_x; //reverses rotation
        //rotiation

        final double lfPow = speed * Math.sin(robotAngle - Math.PI / 4.0) - rightX;
        final double rfPow = speed * Math.cos(robotAngle - Math.PI / 4.0) + rightX;
        final double lbPow = speed * Math.cos(robotAngle - Math.PI / 4.0) - rightX;
        final double rbPow = speed * Math.sin(robotAngle - Math.PI / 4.0) + rightX;
        //determines wheel power


        rf.setPower(rfPow);
        rb.setPower(rbPow);
        lf.setPower(lfPow);
        lb.setPower(lbPow);

        telemetry.addData("Robot Angle", Math.toDegrees(robotAngle));
        telemetry.addData("heading", Math.toDegrees(currentHeading));
        telemetry.addData("anglediff", Math.toDegrees(angleDiff));

    }



    //lf.setPower(0);
    //rf.setPower((0));
    //rb.setPower((0));
    //lb.setPower((0));

    public void Suit_Trig() {
        final double x = gamepad1.left_stick_x;
        final double y = gamepad1.left_stick_y;
        double speed = Math.sqrt(x * x + y * y); //Pythagorean Theorem
        //returns hypotenuse (C value in triangle)


        double robotAngle = Math.atan2(y, x);
        //return angle x (next to center of circle)

        double rightX = gamepad1.right_stick_x; //reverses rotation?
        //rotation

        final double lfPow = speed * Math.sin(robotAngle - Math.PI / 4.0) - rightX;
        final double rfPow = speed * Math.cos(robotAngle - Math.PI / 4.0) + rightX;
        final double lbPow = speed * Math.cos(robotAngle - Math.PI / 4.0) - rightX;
        final double rbPow = speed * Math.sin(robotAngle - Math.PI / 4.0) + rightX;
        //determines wheel power


        rf.setPower(rfPow);
        rb.setPower(rbPow);
        lf.setPower(lfPow);
        lb.setPower(lbPow);

        telemetry.addData("Robot Angle", Math.toDegrees(robotAngle));

    }
    //*/
    //gives wheels wheel power

        /*telemetry.addData("GamepadRx", gamepad1.right_stick_x);
        telemetry.addData("GamepadRy", gamepad1.right_stick_y);
        telemetry.addData("GamepadLy", gamepad1.left_stick_y);*/

    public void noTrig() {
        double drive;
        double strafe;
        double rotate;
        double lfPow;
        double rfPow;
        double lbPow;
        double rbPow;

        drive = -gamepad1.left_stick_y;

        strafe = gamepad1.left_stick_x; //add negative
        rotate = gamepad1.right_stick_x * 0.5;

        lfPow = drive + strafe + rotate;
        lbPow = drive - strafe + rotate;
        rfPow = drive - strafe - rotate;
        rbPow = drive + strafe - rotate;


        lf.setPower(lfPow);
        rf.setPower((rfPow));
        rb.setPower((rbPow));
        lb.setPower((lbPow));


        telemetry.addData("GamepadRx", gamepad1.right_stick_x);
        telemetry.addData("GamepadRy", gamepad1.right_stick_y);
        telemetry.addData("GamepadLy", gamepad1.left_stick_y);

        telemetry.addData("rb", lbPow);
        telemetry.addData("rf", rbPow);
        telemetry.addData("lf", lfPow);
        telemetry.addData("lb", lbPow);
        //*
    }

    public void simpleStrafe() {
        double leftPower;
        double rightPower;

        leftPower = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
        rightPower = Math.abs(gamepad1.right_stick_y) > 0.05 ? gamepad1.right_stick_y : 0;


        if (gamepad1.right_trigger!=0) {
            lf.setPower(-gamepad1.right_trigger*.9);
            lb.setPower(gamepad1.right_trigger *.9);
            rb.setPower(-gamepad1.right_trigger*.9);
            rf.setPower(gamepad1.right_trigger*.9);
        } else if (gamepad1.left_trigger!=0) {
            lf.setPower(gamepad1.left_trigger*.9);
            lb.setPower(-gamepad1.left_trigger *.9);
            rb.setPower(gamepad1.left_trigger*.9);
            rf.setPower(-gamepad1.left_trigger*.9);
        } else {
            lf.setPower(leftPower * .9);
            lb.setPower(leftPower * .9);
            rb.setPower(rightPower * .9);
            rf.setPower(rightPower * .9);
        }
    }

    public void gabeClaw(){ //claw is digital, does not work with 1 and 0 position settings
        if (gamepad1.a) {
            clawR.setPosition(.8); //open claw
            clawL.setPosition(.2);

        }
        else if (gamepad1.b) {
            clawR.setPosition(.47);
            clawL.setPosition(.55); //close claw
        }//else{
        //  clawR.setPosition(1); //open claw
        // clawL.setPosition(0);
        //}

    }

    public void gabeSlide(){
        ls.setPower(gamepad1.right_stick_x);
    }

    public void gabeHook(){

        if (gamepad1.x){
            if (up) {
                hook.setPosition(.9);
            }else{
                hook.setPosition(.15);
            }
            if (hookTime.milliseconds()>400) {
                up = !up;
                hookTime.reset();
            }
        }
    }

    public void claw() {
        /*if (gamepad2.a) {
            clawR.setPosition(.1); //open claw
            clawL.setPosition(.9);
        }
        else if (gamepad2.b) {
            clawR.setPosition(.47); // close claw
            clawL.setPosition(.54);
        }*/
        if (gamepad2.a && clawTime.milliseconds()>500){
            cPos++;
            if (cPos>=3){
                cPos = 0;
            }
            clawTime.reset();
            if (cPos == 0){
                clawR.setPosition(.75); // close claw
                clawL.setPosition(.2);
                telemetry.addLine("Claw Closed");
            }else if (cPos == 1){
                clawR.setPosition(.2); //open claw
                clawL.setPosition(.75);
                telemetry.addLine("Claw Open");
            }else{
                clawR.setPosition(.6); //partial claw
                clawL.setPosition(.4);
                telemetry.addLine("Claw Partial");
            }
        }
        if (cPos == 0){
            telemetry.addLine("Claw Closed");
        }else if (cPos == 1){
            telemetry.addLine("Claw Open");
        }else {
            telemetry.addLine("Claw Partial");
        }
    }

    public void linearSlide() {

        if (gamepad2.right_bumper && slideTime.milliseconds()>500){
            lMod++;
            slideTime.reset();
        }
        if (lMod >=2){ //toggle amount
            lMod = 0;
        }
        if (lMod == 0) {
            ls.setPower(gamepad2.left_stick_y);
        }else if (lMod == 1){
            ls.setPower(gamepad2.left_stick_y * 0.5);
        }
        telemetry.addData("State:", lMod);
    }

    public void hook(){
        /*if (gamepad2.x){
            hook.setPosition(0.9);
        }
        if (gamepad2.y){
            hook.setPosition(0.1);
        }*/
        if (gamepad2.x){
            if (up) {
                hook.setPosition(.9);
            }else{
                hook.setPosition(.1);
            }
            if (hookTime.milliseconds()>500) {
                up = !up;
                hookTime.reset();
            }
        }
    }



    public void resetEncoder() {
        if (gamepad1.x) {
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
        }
    }

    public void disableEncoder(){
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderTelemetry() {

        telemetry.addData("rb",rb.getCurrentPosition());
        telemetry.addData("rf",rf.getCurrentPosition());
        telemetry.addData("lf",lf.getCurrentPosition());
        telemetry.addData("lb",lb.getCurrentPosition());
    }

    /*public void testRevColor(){
        telemetry.addLine()
                .addData("Red", color_sensor.red())   // Red channel value
                .addData("\tGreen", color_sensor.green()) // Green channel value
                .addData("\tBlue", color_sensor.blue());  // Blue channel value

        telemetry.addLine()
                .addData("Luminosity/Alpha",color_sensor.alpha()) // Total luminosity
                .addData("Total", color_sensor.argb())  // Combined color value
                .addData("Hue", hsvValues[0]);
        telemetry.update();
    }

    public void testModernColor(){
        Color.RGBToHSV(color_sensor.red()*8, color_sensor.green()*8, color_sensor.blue()*8, hsvValues);
    }*/

}


//I had one job, but I can't remember that one job ; code the robot_;


