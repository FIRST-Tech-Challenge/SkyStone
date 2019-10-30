package pkg3939;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot3939 {

    public DcMotor RL, RR, FL, FR;
    public Servo servoRight, servoLeft;

    public BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    double  globalAngle;

    public double FLpower, FRpower, RLpower, RRpower;

    private static final double deadZone = 0.10;
    public static final boolean earthIsFlat = true;
    final double reduction = 5;//fine rotation for precise stacking. higher value = slower rotation using triggers

    private final int encoderTicks = 1120;
    private final double wheelDiameter = 3.85827;//in inches

    public void initMotors(HardwareMap hwmap) {
        RL        = hwmap.dcMotor.get("left_drive");
        RR       = hwmap.dcMotor.get("right_drive");
        FL       = hwmap.dcMotor.get("front_left");
        FR      = hwmap.dcMotor.get("front_right");

        RL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initServos(HardwareMap hwmap) {
        servoRight = hwmap.servo.get("servoRight");
        servoLeft = hwmap.servo.get("servoLeft");
    }

    public void initIMU(HardwareMap hwmap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwmap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }

    public void useEncoders(boolean status) {
        if(status) {
            RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setFLPower(double power)
    {
        if(power != FLpower)
        {
            FLpower = power;
            FL.setPower(power);
        }
    }

    public void setRLPower(double power)
    {
        if(power != RLpower)
        {
            RLpower = power;
            RL.setPower(power);
        }
    }

    public void setFRPower(double power)
    {
        if(power != FRpower)
        {
            FRpower = power;
            FR.setPower(power);
        }
    }

    public void setRRPower(double power)
    {
        if(power != RRpower)
        {
            RRpower = power;
            RR.setPower(power);
        }
    }

    public void setAllGivenPower(double power) {
        setFLPower(power);
        setFRPower(power);
        setRRPower(power);
        setRLPower(power);
    }

    public void setAllpower() {
        setFLPower(FLpower);
        setFRPower(FRpower);
        setRRPower(RRpower);
        setRLPower(RLpower);
    }

    public void stopMotors() {
        setAllGivenPower(0);
    }

    public void drive(double LX, double LY, double rotate) {
        if((Math.abs(LX) > deadZone) || (Math.abs(LY) > deadZone) || (Math.abs(rotate) > deadZone)) {
            FLpower = LY - LX + rotate;
            FRpower = LY + LX - rotate;
            RRpower = LY - LX - rotate;
            RLpower = LY + LX + rotate;
        } else if (earthIsFlat) { //stop robot
            FLpower = 0;
            FRpower = 0;
            RRpower = 0;
            RLpower = 0;
        }

        //get max power out of all 4 powers
        double maxPower = Math.max(1.0, Math.max(Math.max(Math.abs(FLpower), Math.abs(RLpower)), Math.max(Math.abs(FRpower), Math.abs(RRpower))));

        //if any of them is greater than 1, it will slow down all by the same ratio
        if(maxPower > 1.0) {
            FLpower /= maxPower;
            FRpower /= maxPower;
            RLpower /= maxPower;
            RRpower /= maxPower;
        }
        FL.setPower(FLpower);
        FR.setPower(FRpower);
        RL.setPower(RLpower);
        RR.setPower(RRpower);
    }

    //for autonomous, use only one axis at a time for now. still under development
    public void driveToPosition(double x, double y, double rotationAngle, double power) {
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        useEncoders(true);

        //y
        double distancePerRotationY = 3.1415 * wheelDiameter; //pi * diameter (inches) = circumference
        double rotationsY = y/distancePerRotationY; //distance / circumference (inches)
        int encoderTargetY = (int)(rotationsY*encoderTicks);

        //x
        double distancePerRotationX = 13.5; //distance per rotations is different than circumference when strafing (inches)
        double rotationsX = x/distancePerRotationX; //distance / circumference (inches)
        int encoderTargetX = (int)(rotationsX*encoderTicks);

        //rotationAngle
        double ticksPerRotation = 0;//measure how many ticks for a 360 rotationAngle
        double rotationsA = rotationAngle/360;
        int encoderTargetA = (int)(rotationsA*ticksPerRotation);

        RL.setTargetPosition(encoderTargetY-encoderTargetX+encoderTargetA);
        RR.setTargetPosition(encoderTargetY+encoderTargetX-encoderTargetA);
        FL.setTargetPosition(encoderTargetY+encoderTargetX+encoderTargetA);
        FR.setTargetPosition(encoderTargetY-encoderTargetX-encoderTargetA);

        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllGivenPower(Math.abs(power));

        stopMotors();

        useEncoders(true);
    }

    public void fineTurn(double LT, double RT) {
        if(LT > deadZone || RT > deadZone){//we don't have to worry about Range.clip here because the abs values will never exceed 1
            FLpower = (-LT + RT)/reduction;
            FRpower = (LT - RT)/reduction;
            RRpower = (LT - RT)/reduction;
            RLpower = (-LT + RT)/reduction;
        }
        setAllpower();
    }

    boolean forks = false;
    boolean aHeld = false;

    public void setForks(boolean aPressed) {
        if (!aHeld && aPressed) {
            aHeld = true;
            forks = !forks;
        } else if (!aPressed)
            aHeld = false;

        if (forks) {
            servoLeft.setPosition(0.5);
            servoRight.setPosition(0.5);
        }
        else if(earthIsFlat)
        {
            servoLeft.setPosition(1);
            servoRight.setPosition(0);
        }

    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            FLpower = power;
            RLpower = power;
            FRpower = -power;
            RRpower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            FLpower = -power;
            RLpower = -power;
            FRpower = power;
            RRpower = power;
        }
        else
            return;

        // set power to rotate.
        setAllpower();

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        }
        else    // left turn.
            while (getAngle() < degrees) {}

        // turn the motors off.
        stopMotors();

        // wait for rotation to stop.
        while(RL.isBusy() || RR.isBusy() || FR.isBusy() || FL.isBusy()) {}

        // reset angle tracking on new heading.
        resetAngle();
    }

}
