package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/* I don't know how to do angular velocity - Akul
Most stuff for turning has been initialized

Most methods that were from last year, were reused. I will change them when i find out all needed values
I took encoder counts per revolution from last year's code. All gear ratios are based off of last year's ratios.


Methods in the class:
Changing motor power
Setting constant velocity
Lynx firmware stuff
Getting current position
Sets motors to RUN_USING_ENCODER
Gets angle
Resets angle
45 degree strafe (2 wheel)
Strafe in any direction (4 wheel)
 */

public class FourWheelMecanumDrivetrain {
    double encoderCountsPerRevolution = 537.6;
    //Motors of the drivetrain
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

    //copy of opmode related objects
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    //Objects for IMU and Rev Hub
    private LynxEmbeddedIMU imu;
    private LynxModule module;

    //IMU related variables for storing states
    private Orientation angles;
    private double lastheading = 0;
    private double deltaheading = 0;
    public double currentheading = 0;

    //Tracking x y coordinate position
    public double x=0;
    public double y=0;
    private double prevleftinches= 0;
    private double prevrightinches=0;

    //variables for velocity
    private double lastLeftVelo = 0;
    private double lastRightVelo = 0;

    //Constructor
    public FourWheelMecanumDrivetrain(HardwareMap hardwareMap, LinearOpMode opMode){

        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        module = hardwareMap.getAll(LynxModule.class).iterator().next();

        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft");

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        imu = new LynxEmbeddedIMU(new FourWheelMecanumDrivetrain.BetterI2cDeviceSynchImplOnSimple(
                new LynxI2cDeviceSynchV2(hardwareMap.appContext, module, 0), true
        ));

        imu.initialize(new BNO055IMU.Parameters());

        opMode.telemetry.addLine("ExH Version: " + getConciseLynxFirmwareVersion(module));
        opMode.telemetry.update();

    }


    private class BetterI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
        private BetterI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
            super(simple, isSimpleOwned);
        }

        @Override
        public void setReadWindow(ReadWindow window) {
            // intentionally do nothing
        }
    }

    private static String getConciseLynxFirmwareVersion(LynxModule module) {
        String rawVersion = module.getFirmwareVersionString();
        String[] parts = rawVersion.split(" ");
        StringBuilder versionBuilder = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            String part = parts[3 + 2*i];
            if (i == 2) {
                versionBuilder.append(part);
            } else {
                versionBuilder.append(part, 0, part.length() - 1);
                versionBuilder.append(".");
            }
        }
        return versionBuilder.toString();
    }


    public void setRightPower(double power){
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }

    public void setLeftPower(double power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
    }
    public void setRobotPower(double power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }

    public int getPosition() {
        return motorFrontRight.getCurrentPosition();
    }

    public double getAngle(){

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        deltaheading = angles.firstAngle - lastheading;

//        opMode.telemetry.addData("delta", deltaheading);

        if (deltaheading < -Math.PI)
            deltaheading += 2*Math.PI ;
        else if (deltaheading >= Math.PI)
            deltaheading -= 2*Math.PI ;

        currentheading += deltaheading;

        lastheading = angles.firstAngle;

        return currentheading;

    }
    public void resetAngle(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        lastheading = angles.firstAngle;
        currentheading = 0;
    }

    public void RunAtConstantVelocity(double x) {

        int initialPos = motorFrontRight.getCurrentPosition();

        double v = 0;

        //creating time object so that kinematics approach can be used
        ElapsedTime time = new ElapsedTime();


        double targetVelocity = (lastLeftVelo+lastRightVelo)/2/537.6*12.56;

        double targetTime = x/targetVelocity;
        time.reset();
        while(time.seconds()<targetTime && opMode.opModeIsActive()){

            motorFrontRight.setVelocity(targetVelocity/12.56*537.6);
            motorBackRight.setVelocity(targetVelocity/12.56*537.6);
            motorFrontLeft.setVelocity(targetVelocity/12.56*537.6);
            motorBackLeft.setVelocity(targetVelocity/12.56*537.6);

        }
        lastRightVelo = motorFrontRight.getVelocity();
        lastLeftVelo = motorFrontLeft.getVelocity();

    }

    public void strafe(double power, double hypotenuseLength, double angle) {
        // resets encoders
        motorFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        double sinDegrees = Math.toDegrees(Math.sin(angle));
        double cosDegrees = Math.toDegrees(Math.cos(angle));

        double distanceRequiredSin = hypotenuseLength*sinDegrees;
        double distanceRequiredCos = hypotenuseLength*cosDegrees;

        // Sets the inches to how many encoder counts are required
        distanceRequiredSin *= encoderCountsPerRevolution;
        distanceRequiredCos *= encoderCountsPerRevolution;

        // creates a variable which calculates the ratio of power needed for each wheel
        double powerRequiredCos = power*cosDegrees;
        double powerRequiredSin = power*sinDegrees;

        // rounds rounds the target position in encoders down (assuming the amount of encoder counts required is a double)
        motorFrontRight.setTargetPosition((int)distanceRequiredSin);
        motorFrontLeft.setTargetPosition((int)distanceRequiredCos);
        motorBackRight.setTargetPosition((int)distanceRequiredCos);
        motorBackLeft.setTargetPosition((int)distanceRequiredSin);

        motorFrontRight.setPower(powerRequiredSin);
        motorFrontLeft.setPower(powerRequiredCos);
        motorBackRight.setPower(powerRequiredCos);
        motorBackLeft.setPower(powerRequiredSin);

        while(motorBackLeft.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorFrontRight.isBusy()) {
            // do nothing
        }

        // sets power to 0
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

    }
    public void strafe45DegreesRight(double power, double inches) {
        motorFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        inches *= encoderCountsPerRevolution;

        motorFrontRight.setTargetPosition((int)inches);
        motorBackLeft.setTargetPosition((int)inches);

        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);

        while(motorFrontRight.isBusy() && motorBackLeft.isBusy() ) {

        }

        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    public void strafe45DegreesLeft(double power, double inches) {
        motorFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        inches *= encoderCountsPerRevolution;

        motorFrontLeft.setTargetPosition((int)inches);
        motorBackRight.setTargetPosition((int)inches);

        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);

        while(motorFrontLeft.isBusy() && motorBackRight.isBusy() ) {

        }

        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
    }
}


