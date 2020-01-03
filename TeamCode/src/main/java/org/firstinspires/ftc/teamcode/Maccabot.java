package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;

public class Maccabot {

    // Pulling in OpMode data
    private OpMode parentOpMode;
    private HardwareMap hardwareMap;
    private double encoder;

    // Drive Motor Variables
    private DcMotor front_left, front_right, back_left, back_right, intake_left, intake_right, lift_left, lift_right;
    private Servo servo;
    private CRServo rack;
    private Servo chad;

    private double liftTargetPos = 0;

    public BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    // Intake Motors TBD
    // private DcMotor intake_left, intake_right;

    public Maccabot(OpMode parentOpMode){
        this.parentOpMode = parentOpMode;
        this.hardwareMap = parentOpMode.hardwareMap;
    }

    public void initializeRobot(){
        parentOpMode.telemetry.addLine("Initializing Drive");
        // Get drive motors
        front_left = hardwareMap.dcMotor.get("front_left"); // Port 0
        front_right = hardwareMap.dcMotor.get("front_right"); // Port 1
        back_left = hardwareMap.dcMotor.get("back_left"); // Port 2
        back_right = hardwareMap.dcMotor.get("back_right"); // Port 3
        intake_left = hardwareMap.dcMotor.get("intake_left");
        intake_right = hardwareMap.dcMotor.get("intake_right");
        lift_left = hardwareMap.dcMotor.get("lift_left");
        lift_right = hardwareMap.dcMotor.get("lift_right");
        servo = hardwareMap.servo.get("servo");
        rack = hardwareMap.crservo.get("bob");
        chad = hardwareMap.servo.get("chad");

        encoder = 0;




        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        intake_left.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO PID!!!
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drive Motors Shouldn't Drive
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();

        param.mode                = BNO055IMU.SensorMode.IMU;
        param.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(param);
    }

    public void drive(double flPower, double frPower, double blPower, double brPower){
        front_left.setPower(flPower);
        front_right.setPower(frPower);
        back_left.setPower(blPower);
        back_right.setPower(brPower);
        // print encoder values
        /*parentOpMode.telemetry.addLine(Integer.toString(front_left.getCurrentPosition()));
        parentOpMode.telemetry.addLine(Integer.toString(front_right.getCurrentPosition()));
        parentOpMode.telemetry.addLine(Integer.toString(back_left.getCurrentPosition()));
        parentOpMode.telemetry.addLine(Integer.toString(back_right.getCurrentPosition()));*/
    }

    public void mecanumDrive(double vtX, double vtY, double vR){
        double flValue = vtY + vtX - vR;
        double frValue = vtY - vtX + vR;
        double blValue = vtY - vtX - vR;
        double brValue = vtY + vtX + vR;

        drive(flValue, frValue, blValue, brValue);
    }

    public void intake(double speed) {
        intake_right.setPower((speed)*.5);//the multiplication of a decimal reduces the motor speed for the INTAKE
        intake_left.setPower((speed)*.5);

       /* parentOpMode.telemetry.addLine(Double.toString(intake_right.getPower()));
        parentOpMode.telemetry.addLine(Double.toString(intake_left.getPower()));*/
    }

    public void servo(double cond1){
        rack.setPower(cond1);
    }

    public void chad(boolean cond1, boolean cond2){
        if (cond1) chad.setPosition(1);
        else if (cond2) chad.setPosition(0);
    }


    public void lift(double lift_up, double lift_down){
        liftTargetPos += 25*lift_up;
        liftTargetPos -= 25*lift_down;
        lift_backend(liftTargetPos, 0.7, 0.004);
    }

    public void lift_backend(double desiredPosition, double maxSpeed, double kP) {
        double currentPosition = lift_right.getCurrentPosition();
        double error = desiredPosition - currentPosition;
        /*if (currentPosition >= 2030 || currentPosition < 0) {
            error = 0;
        }*/
        parentOpMode.telemetry.addData("Current Position", currentPosition);
        parentOpMode.telemetry.addData("Error", error);
        if (currentPosition <= 2020 && currentPosition >= 0) {
            lift_right.setPower(kP*error);
            lift_left.setPower(kP*error);
        } else if (currentPosition > 2020 && error < 0) {
            lift_right.setPower(kP*error);
            lift_left.setPower(kP*error);
        } else if (currentPosition < 0 && error > 0) {
            lift_right.setPower(kP*error);
            lift_left.setPower(kP*error);
        } else {
            lift_right.setPower(0);
            lift_left.setPower(0);
        }
    }

    public void setpower0(){
        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }

    public void auto_forward(int pos, double power){//method for autored for the robot to go forward

        correction = checkDirection();

        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        back_left.setPower(power - correction);
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(power - correction);
        front_left.setTargetPosition(-pos);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(power + correction);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_right.setPower(power + correction);
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();



    }

    public void auto_backward(int pos, double power){//method for autored for the robot to go backward
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        back_left.setPower(-power);//inverted direction for all the motors, so watch out, it goes backwards
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(-power);
        front_left.setTargetPosition(-pos);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(-power);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_right.setPower(-power);
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();


    }
    public void auto_turnright(int pos, double power){//autored method for turn right
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        back_left.setPower(power);
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(power);
        front_left.setTargetPosition(-pos);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(-power);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);//negative power for the right side so the robot does a pinpoint turn to the right

        back_right.setPower(-power);
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();

    }
    public void auto_turnleft(int pos, double power){//autored method for turning left
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        back_left.setPower(-power);//mirror of turn right, but the left motors are negative to pinpoint turn left
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(-power);
        front_left.setTargetPosition(-pos);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(power);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_right.setPower(power);
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();

    }
    public void auto_straferight(int pos, double power){//method for strafing right
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        back_left.setPower(power);
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(-power);
        front_left.setTargetPosition(-pos);//front left inverted to allow for strafing
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(power);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_right.setPower(-power);//back left inverted for strafing
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();
    }
    public void auto_strafeleft(int pos, double power){//mirror of strafe right except with back left and front right inverted for a left strafe.
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);


        back_left.setPower(-power);
        back_left.setTargetPosition(pos);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(power);
        front_left.setTargetPosition(-pos);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(-power);
        front_right.setTargetPosition(-pos);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_right.setPower(power);
        back_right.setTargetPosition(pos);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parentOpMode.telemetry.addData("front-left", front_left.getCurrentPosition());
        parentOpMode.telemetry.addData("back-left", back_left.getCurrentPosition());
        parentOpMode.telemetry.addData("front-right", front_right.getCurrentPosition());
        parentOpMode.telemetry.addData("back-right", back_right.getCurrentPosition());

        parentOpMode.telemetry.update();
    }

    public boolean encoderIsBusy() {
        return (front_right.isBusy());
    }
    public void resetEncoder(){
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runwithoutencoder(){
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void withoutencoder_strafe_left(double power){
        runwithoutencoder();
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        back_left.setPower(-power);
        front_left.setPower(power);
        front_right.setPower(-power);
        back_right.setPower(power);

    }

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
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        drive(leftPower, rightPower, leftPower, rightPower);

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
        setpower0();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public boolean isGyroCalibrated(){return imu.isGyroCalibrated();}

}
