
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;


public class BBSRobot {

    private BBLinearSlide _slide = new BBLinearSlide();
    private BBHooks _hooks = new BBHooks();
    private BBIntake _intakeMotor = new BBIntake();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor centreDrive = null;

    private DcMotor encoder = null;
    private DcMotor rightEncoder = null;

    boolean oN = false;

    private Telemetry telemetry;

    private LinearOpMode _mode;

    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    static final int PULSES_PER_CENTIMETRE = 85;
    static final int PULSES_PER_DEGREE = 51;

    private static double TURN_P = 0.005;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM   = 9.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);



    public void init(HardwareMap hwmap, Telemetry tele, LinearOpMode mode)
    {

        telemetry = tele;
        _intakeMotor.init(hwmap);
        _slide.init(hwmap, tele);
        _hooks.init(hwmap);
        _mode = mode;



        leftDrive  = hwmap.get(DcMotor.class, "left_drive");
        rightDrive = hwmap.get(DcMotor.class, "right_drive");
        centreDrive  = hwmap.get(DcMotor.class, "centre_drive");

        encoder = hwmap.get(DcMotor.class, "encoder");
        rightEncoder = hwmap.get(DcMotor.class, "spool");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        centreDrive.setDirection(DcMotor.Direction.FORWARD);
        encoder.setDirection(DcMotor.Direction.FORWARD);
        rightEncoder.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        centreDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        encoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        runtime.reset();
    }



    public void Move(Gamepad gp1, Gamepad gp2){


        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        //check the gamepads and see what buttons are pressed
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive =  Math.pow(-gp1.left_stick_y,3.0); //Math.log(1.7 * -gp1.left_stick_y + 1);
        double turn  = Math.pow(-gp1.right_stick_x,3.0); //Math.log(1.7 * -gp1.right_stick_x + 1);
        leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower = Range.clip(drive - turn, -1.0, 1.0) ;

        telemetry.addData("leftPower: ", leftPower);
        telemetry.addData("rightPower: ", rightPower);

        centreDrive.setPower(gp1.left_trigger + (-gp1.right_trigger));


        //drivers don't want toggle.

        if(gp2.right_stick_button){
            _slide.SetArmAutoPosAndHold();
        }

        if(gp1.right_bumper || gp2.right_bumper){
            _intakeMotor.Start();
        }

        if(!gp1.right_bumper || !gp2.right_bumper){
            _intakeMotor.Stop();
        }

        if(gp1.left_bumper || gp2.left_bumper){
            _intakeMotor.Reverse();
        }

        if(!gp1.left_bumper || !gp2.left_bumper){
            _intakeMotor.Stop();
        }


        //control the lift arm up and down
        if(gp1.dpad_up){

            _slide.MoveUp();

        }else if(gp1.dpad_down){
            _slide.MoveDown();
        }
        else {
            _slide.StopArm();
        }

        if(gp2.left_stick_y < 0) {
            _slide.MoveUp();
        }
        else if(gp2.left_stick_y > 0){
            _slide.MoveDown();
        }
        else {
            _slide.StopArm();
        }


        //not sure what these should be.
        if(gp1.dpad_right) {
            _slide.Rotate();
        }
        if(gp2.right_stick_x > 0) {
            _slide.Rotate();
        }

        if(gp1.dpad_left) {
            _slide.RotateReset();
        }
        if(gp2.right_stick_x < 0) {
            _slide.RotateReset();
        }

        if(gp1.a || gp2.a) {
            _slide.Grab();
        }

        if(gp1.b || gp2.b){
            _slide.Release();
        }

        if(gp1.x || gp2.x){
            _slide.Level(0.2);
        }else if(gp1.y || gp2.y){
            _slide.ReLevel(0.2);
        }else{
            _slide.LevelStop();
        }

        if(gp2.left_trigger > 0){
            _slide.SlideIn();
        }
        else if(gp2.right_trigger > 0){
            _slide.SlideOut();
        }
        else {
            _slide.StopSlide();
        }
        if (gp1.left_stick_button){
            _hooks.ToggleHook();
            //Add delay
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

        }



        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);


    }

     public void moveForward(int centimetres, double speed){

         encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         telemetry.addData("Enc",  "Setting %7d ", encoder.getCurrentPosition());

        int newTarget = encoder.getCurrentPosition() + (int)(centimetres * PULSES_PER_CENTIMETRE);

         telemetry.addData("Path1",  "Running to %7d ", newTarget);

         telemetry.update();

         encoder.setTargetPosition(newTarget);


        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while (encoder.getCurrentPosition() < newTarget && _mode.opModeIsActive()) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d ", newTarget);

            telemetry.update();
        }

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    public void twoPowerBackwards(int centimetres, double leftSpeed, double rightSpeed){

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Enc",  "Setting %7d ", encoder.getCurrentPosition());

        int newTarget = encoder.getCurrentPosition() - (int)(centimetres * PULSES_PER_CENTIMETRE);

        telemetry.addData("Path1",  "Running to %7d ", newTarget);

        telemetry.update();

        encoder.setTargetPosition(newTarget);


        leftDrive.setPower(-leftSpeed);
        rightDrive.setPower(-rightSpeed);

        while (encoder.getCurrentPosition() > newTarget && _mode.opModeIsActive()) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d ", newTarget);

            telemetry.update();
        }

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    public void moveBackwards(int centimetres, double speed){

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Enc",  "Setting %7d ", encoder.getCurrentPosition());

        int newTarget = encoder.getCurrentPosition() - (int)(centimetres * PULSES_PER_CENTIMETRE);

        telemetry.addData("Path1",  "Running to %7d ", newTarget);

        telemetry.update();

        encoder.setTargetPosition(newTarget);


        leftDrive.setPower(-speed);
        rightDrive.setPower(-speed);

        while (encoder.getCurrentPosition() > newTarget && _mode.opModeIsActive()) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d ", newTarget);

            telemetry.update();
        }

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    public void turnLeft(double degrees, double speed){

        leftEncoderTurn(degrees, speed);
    }

    public void turnRight(double degrees, double speed){

        rightEncoderTurn(degrees, speed);

    }


    private float getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


    private void leftEncoderTurn(double deg, double speed){

        int wheelbase = 20;

        double circum = (wheelbase * 3.14) * 2;

        double centimetersPerDegree = circum / 360;

        double centimetres = deg * centimetersPerDegree;

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Enc",  "Setting %7d ", encoder.getCurrentPosition());

        int newTarget = encoder.getCurrentPosition() + (int)(centimetres * PULSES_PER_CENTIMETRE);

        telemetry.addData("Path1",  "Running to %7d ", newTarget);

        telemetry.update();

        encoder.setTargetPosition(newTarget);


        leftDrive.setPower(speed);
        rightDrive.setPower(-speed);


        while (encoder.getCurrentPosition() < newTarget && _mode.opModeIsActive()) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d ", newTarget);

            telemetry.update();
        }

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    private void rightEncoderTurn(double deg, double speed){

        int wheelbase = 20;

        double circum = (wheelbase * 3.14) * 2;

        double centimetersPerDegree = circum / 360;

        double centimetres = deg * centimetersPerDegree;

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Enc",  "Setting %7d ", encoder.getCurrentPosition());

        int newTarget = encoder.getCurrentPosition() + (int)(centimetres * PULSES_PER_CENTIMETRE);

        telemetry.addData("Path1",  "Running to %7d ", newTarget);

        telemetry.update();

        encoder.setTargetPosition(newTarget);


        leftDrive.setPower(-speed);
        rightDrive.setPower(speed);


        while (Math.abs( encoder.getCurrentPosition()) < Math.abs( newTarget) && _mode.opModeIsActive()) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d ", newTarget);
            telemetry.addData("Current",  "Running to %7d ", encoder.getCurrentPosition());

            telemetry.update();
        }

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    private void gyroTurn(double deg) {
        //TODO: this math is wrong .. i think?
        double target_angle = getHeading() - deg;
        //TODO: make sure we also have a time out on the turn.
        while (Math.abs((target_angle - getHeading())% 360) > 3 && _mode.opModeIsActive()) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute Error
            double motor_output = Range.clip(error_degrees * TURN_P, -.6 ,.6); //Get Correction
            // Send corresponding powers to the motors
            leftDrive.setPower(-motor_output);
            rightDrive.setPower(motor_output);

            Orientation angles = imu.getAngularOrientation (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Spin Target : ",target_angle);
            telemetry.addData("Spin Degree : ",String.format(Locale.getDefault(), "%.1f", angles.firstAngle*-1));
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        //extra 6 seconds to display the values for reading
        while (runtime.milliseconds() < 6000 && _mode.opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Spin Target : ",target_angle);
            telemetry.addData("Spin Degree : ",String.format(Locale.getDefault(), "%.1f", angles.firstAngle*-1));
            telemetry.update();
        }
    }


    public void strafe( double speed){

        centreDrive.setPower(speed);


    }

    public void strafeForTime( double speed, int timeMS){
        centreDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if(speed < 0){
            centreDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else{
            centreDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        centreDrive.setPower(Math.abs(speed));
        _mode.sleep(timeMS);
        centreDrive.setPower(0);

    }


    public void startAutoIntake(){
        _intakeMotor.Start();
    }

}
