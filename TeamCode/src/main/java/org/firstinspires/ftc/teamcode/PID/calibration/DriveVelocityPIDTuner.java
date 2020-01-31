package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.kV;

/*
 * This routine is designed to tune the PID coefficients used by the REV Expansion Hubs for closed-
 * loop velocity control. Although it may seem unnecessary, tuning these coefficients is just as
 * important as the positional parameters. Like the other manual tuning routines, this op mode
 * relies heavily upon the dashboard. To access the dashboard, connect your computer to the RC's
 * WiFi network and navigate to https://192.168.49.1:8080/dash in your browser. Once you've
 * successfully connected, start the program, and your robot will begin moving forward and backward
 * according to a motion profile. Your job is to graph the velocity errors over time and adjust the
 * PID coefficients. Once you've found a satisfactory set of gains, add them to your drive class
 * ctor.
 */
@Config
@Autonomous(name = "DriveVelocityPIDTuner", group = "drive")
public class DriveVelocityPIDTuner extends LinearOpMode {
    public static double DISTANCE = 72;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;
    private PIDCoefficients coefficients;
    private double kV;

    private static final String PID_VAR_NAME = "VELO_PID";

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private String catName;
    private CustomVariable catVar;
    private static String TAG = "DriveVelocityPIDTuner";
    private SampleMecanumDriveBase drive;

    private ArrayList<String> savedData = new ArrayList<>();

    private ArrayList<String> odometryLR = new ArrayList<>();
    private ArrayList<String> wheelLR = new ArrayList<>();
    private ArrayList<String> odometryCenter = new ArrayList<>();
    private ArrayList<String> imuAngle = new ArrayList<>();
    private ArrayList<String> targetAndError = new ArrayList<>();

    private int index = 0;

    private HardwareMap hwMap;

    private static MotionProfile generateProfile(boolean movingForward) {
        DriveConstantsPID.updateConstantsFromProperties();
        DISTANCE = DriveConstantsPID.TEST_DISTANCE;
        RobotLogger.dd(TAG, "DISTANCE: "+Double.toString(DISTANCE));
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
                BASE_CONSTRAINTS.maxVel,
                BASE_CONSTRAINTS.maxAccel,
                BASE_CONSTRAINTS.maxJerk);
    }

    private void addPidVariable() {
        catName = getClass().getSimpleName();
        catVar = (CustomVariable) dashboard.getConfigRoot().getVariable(catName);
        if (catVar == null) {
            // this should never happen...
            catVar = new CustomVariable();
            dashboard.getConfigRoot().putVariable(catName, catVar);

            RobotLogger.dd("", "Unable to find top-level category %s", catName);
        }

        CustomVariable pidVar = new CustomVariable();
        pidVar.putVariable("kP", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).kP;
            }

            @Override
            public void set(Double value) {
                PIDCoefficients coeffs = drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDCoefficients(value, coeffs.kI, coeffs.kD));
            }
        }));
        pidVar.putVariable("kI", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).kI;
            }

            @Override
            public void set(Double value) {
                PIDCoefficients coeffs = drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDCoefficients(coeffs.kP, value, coeffs.kD));
            }
        }));
        pidVar.putVariable("kD", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).kD;
            }

            @Override
            public void set(Double value) {
                PIDCoefficients coeffs = drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDCoefficients(coeffs.kP, coeffs.kI, value));
            }
        }));

        catVar.putVariable(PID_VAR_NAME, pidVar);
        dashboard.updateConfig();
    }

    private void removePidVariable() {
        if (catVar.size() > 1) {
            catVar.removeVariable(PID_VAR_NAME);
        } else {
            dashboard.getConfigRoot().removeVariable(catName);
        }
        dashboard.updateConfig();
    }

    @Override
    public void runOpMode() {
        hwMap = new HardwareMap(hardwareMap);

        DriveConstantsPID.updateConstantsFromProperties();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        if (!RUN_USING_ENCODER) {
            RobotLogger.dd("%s does not need to be run if the built-in motor velocity" +
                    "PID is not in use", getClass().getSimpleName());
        }
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        PIDCoefficients oldPID = new PIDCoefficients(DriveConstantsPID.MOTOR_VELO_PID.kP, DriveConstantsPID.MOTOR_VELO_PID.kI,
                DriveConstantsPID.MOTOR_VELO_PID.kD);
        double oldkV = DriveConstantsPID.kV;
        ;

        coefficients = new PIDCoefficients(DriveConstantsPID.MOTOR_VELO_PID.kP, DriveConstantsPID.MOTOR_VELO_PID.kI,
                DriveConstantsPID.MOTOR_VELO_PID.kD);
        kV = DriveConstantsPID.kV;


        int selected = 0;
        boolean blocker1 = false;
        boolean blocker2 = false;
        boolean blocker3 = false;

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        if (DriveConstantsPID.USING_BULK_READ == false)
            drive = new SampleMecanumDriveREV(hardwareMap, false);
        else
            drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);

        addPidVariable();

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();


        while (!isStopRequested()) {
            // calculate and set the motor power
            double profileTime = clock.seconds() - profileStart;

            if (profileTime > activeProfile.duration()) {
                // generate a new profile
                movingForwards = !movingForwards;
                activeProfile = generateProfile(movingForwards);
                profileStart = clock.seconds();
            }

            MotionState motionState = activeProfile.get(profileTime);
            double targetPower = kV * motionState.getV();
            drive.setDrivePower(new Pose2d(targetPower, 0, 0));

            List<Double> velocities = drive.getWheelVelocities();
            List<Double> positions = drive.getWheelPositions();
            RobotLogger.dd(TAG, "getWheelVelocities");
            drive.print_list_double(velocities);
            RobotLogger.dd(TAG, "getWheelPositions");
            drive.print_list_double(positions);

            // update telemetry
            telemetry.addData("targetVelocity", motionState.getV());
            RobotLogger.dd(TAG, "targetVelocity " + Double.toString(motionState.getV()));
            for (int i = 0; i < velocities.size(); i++) {
                telemetry.addData("velocity" + i, velocities.get(i));
                telemetry.addData("error" + i, motionState.getV() - velocities.get(i));
                RobotLogger.dd(TAG, "velocity " + i + " " + Double.toString(velocities.get(i)));
                RobotLogger.dd(TAG, "error " + i + " " + Double.toString(motionState.getV() - velocities.get(i)));

            }

            targetAndError.add(System.currentTimeMillis() + ": Distance: " + DISTANCE + ", PID: " + coefficients + ", kV: "
                    + kV + " ---====--- TargetVelocity: " +  motionState.getV() + " || CurrentVelocities:");
            for (int i = 0; i < velocities.size(); i++) {
                targetAndError.set(index, targetAndError.get(index) + " " + i + ": " + velocities.get(i));
            }

            targetAndError.set(index, targetAndError.get(index) + " || Errors:");

            for(int i = 0; i < velocities.size(); i++){
                targetAndError.set(index, targetAndError.get(index) + " " + i + ": " + (motionState.getV() - velocities.get(i)));
            }

            targetAndError.set(index, targetAndError.get(index) + "\n");

            index += 1;

            /*savedData.add("\n" + "FrontLeftWheel: " + hwMap.frontLeft.getCurrentPosition() + ", FrontRightWheel: " +
                    hwMap.frontRight.getCurrentPosition() + ", BackLeftWheel: " + hwMap.backLeft.getCurrentPosition() +
                    ", BackRightWheel: " + hwMap.backRight.getCurrentPosition() + ", FrontLeftOdo: " +
                    hwMap.leftIntake.getCurrentPosition() + ", FrontRightOdo: " + hwMap.liftTwo.getCurrentPosition() +
                    ", BackMiddleOdo: " + hwMap.rightIntake.getCurrentPosition() + ", IMU Angle (Rad): " +
                    imu.getAngularOrientation().firstAngle + ", IMU Angle (Deg): " +
                    Math.toDegrees(imu.getAngularOrientation().firstAngle) +
                    " || CurrentVelocities: " + velocities + ", TargetVelocities: " + motionState.getV() + " || MotorVeloPID: " +
                    DriveConstantsPID.MOTOR_VELO_PID);

            odometryCenter.add(hwMap.rightIntake.getCurrentPosition() + "\n");
            imuAngle.add(imu.getAngularOrientation().firstAngle + "\n");
            odometryLR.add(hwMap.leftIntake.getCurrentPosition() + "," + hwMap.liftTwo.getCurrentPosition() + "\n");
            wheelLR.add(hwMap.frontLeft.getCurrentPosition() + "," + hwMap.backLeft.getCurrentPosition() + "," +
                    hwMap.frontRight.getCurrentPosition() + "," + hwMap.backRight.getCurrentPosition() + "\n");*/

            if (gamepad1.right_bumper) {
                DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/DriveVelocityPIDTunerConstants_" +
                        System.currentTimeMillis() + ".txt", "Distance: " + DISTANCE + ", PID: " + coefficients + ", kV: "
                        + kV);

                String targetError = targetAndError.toString();
                targetError = targetError.replaceAll("]", "");
                targetError = targetError.replaceAll("\\[", "");

                DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/DriveVelocityPIDTunerTargetsAndErrors_" +
                        System.currentTimeMillis() + ".txt", targetError);
                break;
            }

            if (gamepad1.left_stick_y >= 0.5) {
                DISTANCE -= 1;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            } else if (gamepad1.left_stick_y <= -0.5) {
                DISTANCE += 1;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            }

            if(DISTANCE < 0)
                DISTANCE = 0;

            if (gamepad1.right_stick_y >= 0.5) {
                if (selected == 0) {
                    coefficients.kP -= 0.5;
                    try{
                        Thread.sleep(100);
                    } catch (Exception e){}
                }else if (selected == 1) {
                    coefficients.kI -= 0.5;
                    try{
                        Thread.sleep(100);
                    } catch (Exception e){}
                }else if (selected == 2) {
                    coefficients.kD -= 0.5;
                    try{
                        Thread.sleep(100);
                    } catch (Exception e){}
                }else if (selected == 3) {
                    kV -= 0.001;
                    try{
                        Thread.sleep(100);
                    } catch (Exception e){}
                }
            } else if (gamepad1.right_stick_y <= -0.5) {
                if (selected == 0) {
                    coefficients.kP += 0.5;
                    try{
                        Thread.sleep(100);
                    } catch (Exception e){}
                }else if (selected == 1) {
                    coefficients.kI += 0.5;
                    try{
                        Thread.sleep(100);
                    } catch (Exception e){}
                }else if (selected == 2) {
                    coefficients.kD += 0.5;
                    try{
                        Thread.sleep(100);
                    } catch (Exception e){}
                }else if (selected == 3) {
                    kV += 0.001;
                    try{
                        Thread.sleep(100);
                    } catch (Exception e){}
                }
            }

            if (gamepad1.dpad_right && !blocker1) {
                if (selected < 3)
                    selected += 1;
                blocker1 = true;
            } else if (!gamepad1.dpad_right && blocker1) {
                blocker1 = false;
            }

            if (gamepad1.dpad_left && !blocker2) {
                if (selected > 0)
                    selected -= 1;
                blocker2 = true;
            } else if (!gamepad1.dpad_right && blocker2) {
                blocker2 = false;
            }

            if (gamepad1.left_bumper) {
                drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
                DriveConstantsPID.kV = kV;
            }

            telemetry.addData("Instructions", "A to begin moving. Press R bumper to save data. " + "DPAD L & R to select. L stick to change distance. R stick to " +
                    "change selected value. L bumper to save values.");

            if(selected == 0)
                telemetry.addData("SELECTED", "P");
            else if(selected == 1)
                telemetry.addData("SELECTED", "I");
            else if(selected == 2)
                telemetry.addData("SELECTED", "D");
            else if(selected == 3)
                telemetry.addData("SELECTED", "kV");

            telemetry.addData("DISTANCE", DISTANCE);
            telemetry.addData("PID", coefficients);
            telemetry.addData("kV", kV);
            telemetry.addData("ERROR", drive.getLastError());
            telemetry.update();
        }

        /*String temp = odometryCenter.toString().replaceAll(",","");
        temp = temp.replaceAll("\\[","");
        temp = temp.replaceAll("]","");
        DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/OdometryCenter_" +
                System.currentTimeMillis() + ".csv", temp);

        temp = imuAngle.toString().replaceAll(",","");
        temp = temp.replaceAll("\\[","");
        temp = temp.replaceAll("]","");
        DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/ImuAngle_" +
                System.currentTimeMillis() + ".csv", temp);

        temp = odometryLR.toString().replaceAll("\\[","");
        temp = temp.replaceAll("]","");
        int counter = 0;
        for(int i = 0; i < temp.length(); i++){
            if(temp.charAt(i) == ','){
                counter += 1;
            }

            if(counter == 2) {
                temp = temp.substring(0, i) + temp.substring(i + 1);
                counter = 0;
            }
        }
        DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/OdometryLR_" +
                System.currentTimeMillis() + ".csv", temp);

        temp = wheelLR.toString().replaceAll("\\[","");
        temp = temp.replaceAll("]","");
        counter = 0;
        for(int i = 0; i < temp.length(); i++){
            if(temp.charAt(i) == ','){
                counter += 1;
            }

            if(counter == 4) {
                temp = temp.substring(0, i) + temp.substring(i + 1);
                counter = 0;
            }
        }
        DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/WheelLR_" +
                System.currentTimeMillis() + ".csv", temp);

        DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/DriveVelRAWData_" +
                System.currentTimeMillis() + ".txt", savedData.toString());*/

        drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, oldPID);
        DriveConstantsPID.kV = oldkV;

        removePidVariable();
    }
}
