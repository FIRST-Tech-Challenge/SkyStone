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
    private DcMotor RL, RR, FL, FR;
    private Servo servoRight, servoLeft;

    public BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    double  globalAngle, correction;

    private final int encoderTicks = 1120;
    private final double wheelDiameter = 3.85827;//in inches

    public void init(HardwareMap hwmap) {
        RL        = hwmap.dcMotor.get("left_drive");
        RR       = hwmap.dcMotor.get("right_drive");
        FL       = hwmap.dcMotor.get("front_left");
        FR      = hwmap.dcMotor.get("front_right");
        servoRight = hwmap.servo.get("servoRight");
        servoLeft = hwmap.servo.get("servoLeft");


        RL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

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

    public void move(double x, double y, double rotationAngle, double power) {
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        RL.setPower(Math.abs(power));//childproof. must have always positive power
        RR.setPower(Math.abs(power));
        FL.setPower(Math.abs(power));
        FR.setPower(Math.abs(power));

        RL.setPower(0);
        RR.setPower(0);
        FL.setPower(0);
        FR.setPower(0);

        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
//    private void rotate(int degrees, double power)
//    {
//        double  leftPower, rightPower;
//
//        // restart imu movement tracking.
//        resetAngle();
//
//        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//        // clockwise (right).
//
//        if (degrees < 0)
//        {   // turn right.
//            leftPower = power;
//            rightPower = -power;
//        }
//        else if (degrees > 0)
//        {   // turn left.
//            leftPower = -power;
//            rightPower = power;
//        }
//        else return;
//
//        // set power to rotate.
//        leftMotor.setPower(leftPower);
//        rightMotor.setPower(rightPower);
//
//        // rotate until turn is completed.
//        if (degrees < 0)
//        {
//            // On right turn we have to get off zero first.
//            while (opModeIsActive() && getAngle() == 0) {}
//
//            while (opModeIsActive() && getAngle() > degrees) {}
//        }
//        else    // left turn.
//            while (opModeIsActive() && getAngle() < degrees) {}
//
//        // turn the motors off.
//        rightMotor.setPower(0);
//        leftMotor.setPower(0);
//
//        // wait for rotation to stop.
//        sleep(1000);
//
//        // reset angle tracking on new heading.
//        resetAngle();
//    }

}
