
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.common.math.TimePose;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.Locale;


public class BBSRobot {

    private BBHooks _hooks = new BBHooks();
    private BBIntake _intakeMotor = new BBIntake();

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotor leftRear = null;

    private ExpansionHubEx chassisHub1;
    private ExpansionHubEx chassisHub2;

    public TwoWheelTrackingLocalizer localizer;
    public RevBulkData lastChassis1Read;
    public RevBulkData lastChassis2Read;
    public double lastHeading;
    private double headingOffset;

    private Telemetry telemetry;

    private LinearOpMode _mode;

    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;



    public void init(HardwareMap hwmap, Telemetry tele, LinearOpMode mode)
    {

        telemetry = tele;
        _intakeMotor.init(hwmap);

        _hooks.init(hwmap);
        _mode = mode;



        leftFront = hwmap.get(DcMotor.class, "left_front");
        rightFront = hwmap.get(DcMotor.class, "right_front");

        rightRear = hwmap.get(DcMotor.class, "right_rear");
        leftRear = hwmap.get(DcMotor.class, "left_rear");

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        imu = hwmap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        //remap axis
        headingOffset = imu.getAngularOrientation().firstAngle;


        runtime.reset();
        Pose start = new Pose(0,0,0);
        TimePose startPose4D = new TimePose(start, System.currentTimeMillis());
        localizer = new TwoWheelTrackingLocalizer(0, 1, startPose4D);
        this.lastHeading = 0;

        chassisHub1 = hwmap.get(ExpansionHubEx.class, "Expansion Hub 1");
        chassisHub2 = hwmap.get(ExpansionHubEx.class, "Expansion Hub 2");
    }

    public void LocalizerUpdate(){
        this.lastChassis1Read = chassisHub1.getBulkInputData();
        this.lastChassis2Read = chassisHub2.getBulkInputData();
        this.lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        localizer.update(lastChassis1Read, lastChassis2Read, lastHeading);
    }

    public void Move(Gamepad gp1, Gamepad gp2){



        this.lastChassis1Read = chassisHub1.getBulkInputData();
        this.lastChassis2Read = chassisHub2.getBulkInputData();
        this.lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        localizer.update(lastChassis1Read, lastChassis2Read, lastHeading);

        telemetry.addData("X:", String.format("%.1f", localizer.x()));
        telemetry.addData("Y:",String.format("%.1f", localizer.y()));
        telemetry.addData("H:",String.format("%.1f", MathUtil.angleWrap(Math.toDegrees(localizer.h()))));
        telemetry.update();

        double slowScale = ((1 - gp1.left_trigger) * 0.7 + 0.3);
        double leftX = MecanumUtil.deadZone(gp1.left_stick_x, 0.05) * slowScale;
        double leftY = MecanumUtil.deadZone(gp1.left_stick_y, 0.05) * slowScale;
        double angle = Math.atan2(leftY, leftX) + Math.PI / 2;

        double driveScale = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        driveScale = Range.clip(driveScale, 0, 1);

        // Exponentiate our turn
        double turn = Math.copySign(
                Math.pow(MecanumUtil.deadZone(gp1.right_stick_x, 0.05), 2),
                gp1.right_stick_x) * slowScale;


        MecanumPowers powers = MecanumUtil.powersFromAngle(angle, driveScale, turn);
        setPowers(powers);

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


        if (gp1.left_stick_button){
            _hooks.ToggleHook();
            //Add delay
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

        }


       /* telemetry.addData("1:", leftRear.getCurrentPosition()); //side
        telemetry.addData("2", rightRear.getCurrentPosition()); //forward
        telemetry.update();*/

    }

    private void setPowers(MecanumPowers powers)
    {
        leftFront.setPower(powers.frontLeft);
        rightFront.setPower(powers.frontRight);
        leftRear.setPower(powers.backLeft);
        rightRear.setPower(powers.backRight);


    }


    public static Pose SLIP_DISTANCES = new Pose(22, 5, Math.toRadians(30));
    public static double MIN_TRANSLATION_POWERS = 0.2;
    public static double CUTOFF_MOTOR_POWER = 0.05;
    public static Pose GUNNING_REDUCTION_DISTANCES = new Pose(6, 6, Math.PI/2);
    public static Pose FINE_REDUCTION_DISTANCES = new Pose(3, 3, Math.PI);

     public void RobotMove(Waypoint target, double movementSpeed) {

         telemetry.addData("X:", String.format("%.1f", localizer.x()));
         telemetry.addData("Y:",String.format("%.1f", localizer.y()));
         telemetry.addData("H:",String.format("%.1f", MathUtil.angleWrap(Math.toDegrees(localizer.h()))));


        //we can use our localiser to move to the position that we want.
         double distance = target.minus(localizer.pose()).radius();
         double relAngle = localizer.pose().minus(target).atan() - localizer.pose().heading;
         double relX = distance * Math.cos(relAngle);
         double relY = distance * Math.sin(relAngle);

         Pose translationPowers = new Pose(relX, relY, 0).scale(movementSpeed);

         Pose reductionDistances = FINE_REDUCTION_DISTANCES;
         translationPowers.x /= reductionDistances.x;
         translationPowers.y /= reductionDistances.y;

         // Ensure we're moving x/y a little unless we're stopped
         if (translationPowers.radius() < MIN_TRANSLATION_POWERS) {
             translationPowers.scale(MIN_TRANSLATION_POWERS / translationPowers.radius());
         }

         telemetry.addData("distance:", String.format("%.1f", distance));
         telemetry.addData("relAngle:", String.format("%.1f", relAngle));

         double forwardAngle = target.minus(localizer.pose()).atan();
         double backwardAngle = forwardAngle + Math.PI;
         double angleToForward = MathUtil.angleWrap(forwardAngle - localizer.pose().heading);
         double angleToBackward = MathUtil.angleWrap(backwardAngle - localizer.pose().heading);
         double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
         double desiredAngle = autoAngle;

         // We'll do a quadratic decay to zero to only correct for really big heading errors
         double angleToTarget = MathUtil.angleWrap(desiredAngle - localizer.pose().heading);
         translationPowers.heading = angleToTarget / reductionDistances.heading;

         telemetry.addData("angleToTarget:", String.format("%.1f", angleToTarget));

         telemetry.addData("autoAngle:", String.format("%.1f", autoAngle));
         telemetry.addData("t1x:", String.format("%.1f", translationPowers.x));
         telemetry.addData("t1y:", String.format("%.1f", translationPowers.y));
         telemetry.addData("t1h:", String.format("%.1f", translationPowers.heading));
         //power the wheels
         //setPowers(new MecanumPowers(translationPowers));

         telemetry.update();
    }

    public void RobotMoveY(Waypoint target, double movementSpeed) {

        //TODO: Think about this -- we loose position on the field
       /*rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        telemetry.addData("X:", String.format("%.1f", localizer.x()));
        telemetry.addData("Y:",String.format("%.1f", localizer.y()));
        telemetry.addData("H:",String.format("%.1f", Math.toDegrees(localizer.h())));
        telemetry.update();

        while(_mode.opModeIsActive() && localizer.y() < target.y) {
            LocalizerUpdate();
            MecanumPowers powers = MecanumUtil.powersFromAngle(0, movementSpeed, 0);
            setPowers(powers);
            telemetry.addLine("Robot Y");
            telemetry.addData("X:", String.format("%.1f", localizer.x()));
            telemetry.addData("Y:",String.format("%.1f", localizer.y()));
            telemetry.addData("H:",String.format("%.1f", MathUtil.angleWrap(Math.toDegrees(localizer.h()))));
            telemetry.update();
        }
    }

    public void RobotMoveX(Waypoint target, double movementSpeed) {

         //TODO: Think about this
        /*rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        telemetry.addData("X:", String.format("%.1f", localizer.x()));
        telemetry.addData("Y:",String.format("%.1f", localizer.y()));
        telemetry.addData("H:",String.format("%.1f", Math.toDegrees(localizer.h())));
        telemetry.update();

        while(_mode.opModeIsActive() && Math.abs(localizer.x()) < Math.abs(target.x)) {
            LocalizerUpdate();

            double slowScale = 0.3;

            double direction = 1;

            if(target.x  < 0){
                direction = -1;
            }

            // Exponentiate our turn
            double turn = Math.copySign(
                    Math.pow(MecanumUtil.deadZone(direction, 0.05), 2),
                    direction) * slowScale;

            MecanumPowers powers = MecanumUtil.powersFromAngle(0, 0, turn);
            setPowers(powers);
            telemetry.addLine("Robot X");
            telemetry.addData("X:", String.format("%.1f", localizer.x()));
            telemetry.addData("Y:",String.format("%.1f", localizer.y()));
            telemetry.addData("H:",String.format("%.1f", MathUtil.angleWrap(Math.toDegrees(localizer.h()))));
            telemetry.update();
        }
    }

    public Pose pose(){
         return localizer.pose();
    }

    public void Stop(){
         setPowers(new MecanumPowers(0,0,0,0));
    }

    public void moveBackwards(int centimetres, double speed){

    }

    public void moveForward(int centimetres, double speed){

    }


    public void turnLeft(double degrees, double speed){


    }

    public void turnRight(double degrees, double speed){



    }



}
