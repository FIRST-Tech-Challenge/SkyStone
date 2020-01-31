package org.firstinspires.ftc.teamcode.PID.mecanum;

import android.util.TimingLogger;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.util.DashboardUtil;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.ROTATION_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.STRAFE_BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.PID.util.DashboardUtil.drawRobot;
import static org.firstinspires.ftc.teamcode.PID.util.DashboardUtil.drawSampledPath;

/*
 * Base class with shared functionality for sample mecanum drives. All hardware-specific details are
 * handled in subclasses.
 */
@Config
public abstract class SampleMecanumDriveBase extends MecanumDrive {
    public static PIDCoefficients xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.txP, DriveConstantsPID.txI, DriveConstantsPID.txD);
    public static PIDCoefficients yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.tyP, DriveConstantsPID.tyI, DriveConstantsPID.tyD);
    public static PIDCoefficients HEADING_PID  = new PIDCoefficients(DriveConstantsPID.hP, DriveConstantsPID.hI, DriveConstantsPID.hD);    //3, 0, 0


    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    public TrajectoryFollower follower;

    private List<Double> lastWheelPositions;
    private double lastTimestamp;
    private static String TAG = "SampleMecanumDriveBase";
    private boolean strafe = false;

    public SampleMecanumDriveBase() {
        super(DriveConstantsPID.kV, DriveConstantsPID.kA, DriveConstantsPID.kStatic, TRACK_WIDTH);
        createControllers();
    }
    public SampleMecanumDriveBase(boolean s){
        super(DriveConstantsPID.kV, DriveConstantsPID.kA, DriveConstantsPID.kStatic, TRACK_WIDTH);
        strafe = s;
        createControllers();
    }
    public void createControllers()
    {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        if (strafe == false) {
            RobotLogger.dd(TAG, "using non-strafing PID, maxVel: %f, maxAccl: %f", BASE_CONSTRAINTS.maxVel, BASE_CONSTRAINTS.maxAccel);
            xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.txP, DriveConstantsPID.txI, DriveConstantsPID.txD);
            yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.tyP, DriveConstantsPID.tyI, DriveConstantsPID.tyD);
            HEADING_PID = new PIDCoefficients(DriveConstantsPID.hP, DriveConstantsPID.hI, DriveConstantsPID.hD);

            constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        }
        else
        {
            RobotLogger.dd(TAG, "using strafing PID, maxVel: %f, maxAccl: %f", STRAFE_BASE_CONSTRAINTS.maxVel, STRAFE_BASE_CONSTRAINTS.maxAccel);
            xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.stxP, DriveConstantsPID.stxI, DriveConstantsPID.stxD);
            yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.styP, DriveConstantsPID.styI, DriveConstantsPID.styD);
            HEADING_PID = new PIDCoefficients(DriveConstantsPID.shP, DriveConstantsPID.shI, DriveConstantsPID.shD);

            constraints = new MecanumConstraints(STRAFE_BASE_CONSTRAINTS, TRACK_WIDTH);

        }
        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        follower = new HolonomicPIDVAFollower(xTRANSLATIONAL_PID, yTRANSLATIONAL_PID, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();
        RobotLogger.dd(TAG, "turn: current heading "+Double.toString(heading)+" angle "+Double.toString(angle));
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turnSync(double angle) {
        turn(angle);
        waitForIdle();
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectorySync(Trajectory trajectory) {
        followTrajectory(trajectory);
        waitForIdle();
    }

    public void resetFollowerWithParameters(boolean strafe, boolean rotating){
        addSpacer();
        RobotLogger.dd("Pre-Reinstantiate Error", follower.getLastError() + "");
        RobotLogger.dd("Follower PID Constants", strafe ? "STRAFE" : "BASE");
        if (!strafe) {
            RobotLogger.dd(TAG, "using non-strafing PID, maxVel: %f, maxAccl: %f", BASE_CONSTRAINTS.maxVel, BASE_CONSTRAINTS.maxAccel);
            xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.txP, DriveConstantsPID.txI, DriveConstantsPID.txD);
            yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.tyP, DriveConstantsPID.tyI, DriveConstantsPID.tyD);
            HEADING_PID = new PIDCoefficients(DriveConstantsPID.hP, DriveConstantsPID.hI, DriveConstantsPID.hD);

            constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        }
        else
        {
            RobotLogger.dd(TAG, "using strafing PID, maxVel: %f, maxAccl: %f", STRAFE_BASE_CONSTRAINTS.maxVel, STRAFE_BASE_CONSTRAINTS.maxAccel);
            xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.stxP, DriveConstantsPID.stxI, DriveConstantsPID.stxD);
            yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.styP, DriveConstantsPID.styI, DriveConstantsPID.styD);
            HEADING_PID = new PIDCoefficients(DriveConstantsPID.shP, DriveConstantsPID.shI, DriveConstantsPID.shD);

            constraints = new MecanumConstraints(STRAFE_BASE_CONSTRAINTS, TRACK_WIDTH);

        }

        if(rotating){
            xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.txP, DriveConstantsPID.txI, DriveConstantsPID.txD);
            yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.tyP, DriveConstantsPID.tyI, DriveConstantsPID.tyD);
            HEADING_PID = new PIDCoefficients(DriveConstantsPID.hP, DriveConstantsPID.hI, DriveConstantsPID.hD);

            constraints = new MecanumConstraints(ROTATION_CONSTRAINTS, TRACK_WIDTH);
        }

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);
        follower = new HolonomicPIDVAFollower(xTRANSLATIONAL_PID, yTRANSLATIONAL_PID, HEADING_PID);
        RobotLogger.dd("STATUS", "Re-Inited Follower");
        RobotLogger.dd("Post-Reinstantiate Error", follower.getLastError() + "");
        addSpacer();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        RobotLogger.dd(TAG, "roadrunner control loop starts");
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        RobotLogger.dd(TAG, "update: x " + currentPose.getX());
        RobotLogger.dd(TAG, "y " + currentPose.getY());
        RobotLogger.dd(TAG, "heading " + Double.toString(currentPose.getHeading()));

        RobotLogger.dd(TAG, "xError " + lastError.getX());
        RobotLogger.dd(TAG, "yError " + lastError.getY());
        RobotLogger.dd(TAG, "headingError "  + lastError.getHeading());

        drawPosition(packet, currentPose);

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                double correction = turnController.update(currentPose.getHeading(), targetOmega);
                RobotLogger.dd(TAG, "TURN: targetOmega "+Double.toString(targetOmega)+" targetAlpha "+Double.toString(targetAlpha));
                RobotLogger.dd(TAG, "correction "+Double.toString(correction));
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                drawSampledPath(fieldOverlay, trajectory.getPath());

                fieldOverlay.setStroke("#F44336");
                double t = follower.elapsedTime();
                drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }
        dashboard.sendTelemetryPacket(packet);

    }
    /// new function added;
    public abstract void setBrakeonZeroPower(boolean flag);

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public List<Double> getWheelVelocities() {
        List<Double> positions = getWheelPositions();
        double currentTimestamp = clock.seconds();

        List<Double> velocities = new ArrayList<>(positions.size());;
        if (lastWheelPositions != null) {
            double dt = currentTimestamp - lastTimestamp;
            for (int i = 0; i < positions.size(); i++) {
                velocities.add((positions.get(i) - lastWheelPositions.get(i)) / dt);
            }
        } else {
            for (int i = 0; i < positions.size(); i++) {
                velocities.add(0.0);
            }
        }
        String t="";
        for (int i = 0; i < lastWheelPositions.size(); i ++)
        {
            t=t+Double.toString(lastWheelPositions.get(i)) + "\t";
        }
        RobotLogger.dd(TAG,"last ts: "+Double.toString(lastTimestamp)+" last wheel position: "+t);
        t="";
        for (int i = 0; i < positions.size(); i ++)
        {
            t=t+Double.toString(positions.get(i)) + "\t";
        }
        RobotLogger.dd(TAG,"current ts: "+Double.toString(currentTimestamp)+" current wheel position: "+t);
        t="";
        for (int i = 0; i < velocities.size(); i ++)
        {
            t=t+Double.toString(velocities.get(i)) + "\t";
        }
        RobotLogger.dd(TAG, "velocity: "+t);

        lastTimestamp = currentTimestamp;
        lastWheelPositions = positions;

        return velocities;
    }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);
    public void print_list_double(List<Double> list){
        //motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        int wheel_num = list.size();
        if (wheel_num == 4) {
            for (int i = 0; i < list.size(); i++) {
                String wheel_name = "";
                if (i == 0)
                    wheel_name = "leftFront";
                else if (i == 1)
                    wheel_name = "leftRear";
                else if (i == 2)
                    wheel_name = "rightRear";
                else if (i == 3)
                    wheel_name = "rightFront";
                else
                    wheel_name = "unexpected wheel name";

                RobotLogger.dd(TAG, wheel_name + "  " + Double.toString(list.get(i)));
            }
        } else if (wheel_num == 3)
        {
            for (int i = 0; i < list.size(); i++) {
                String wheel_name = "";
                if (i == 0)
                    wheel_name = "leftOdom";
                else if (i == 1)
                    wheel_name = "rightOdom";
                else if (i == 2)
                    wheel_name = "frontOdom";

                RobotLogger.dd(TAG, wheel_name + "  " + Double.toString(list.get(i)));
            }
        }
        else
        {
            for (int i = 0; i < list.size(); i++) {
                String wheel_name = "";
                RobotLogger.dd(TAG, wheel_name + "  " + Double.toString(list.get(i)));
            }
        }
    }

    public static void drawSampledTrajectory(Trajectory trajectory) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        DashboardUtil.drawSampledTrajectory(packet.fieldOverlay(), trajectory);
        dashboard.sendTelemetryPacket(packet);
    }

    public static void drawPosition(SampleMecanumDriveBase drive){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        DashboardUtil.drawRobot(packet.fieldOverlay(), drive.getPoseEstimate());
        dashboard.sendTelemetryPacket(packet);
    }

    private static void drawPosition(TelemetryPacket packet, Pose2d poseEstimate){
        DashboardUtil.drawRobot(packet.fieldOverlay(), poseEstimate);
    }

    private static void addSpacer(){
        for(int i = 0; i < 10; i++)
            RobotLogger.dd("", "=========--+--=========");
    }
}
