package org.firstinspires.ftc.teamcode.PID.mecanum;

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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.util.DashboardUtil;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.BASE_CONSTRAINTS;
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
    private TrajectoryFollower follower;

    private List<Double> lastWheelPositions;
    private double lastTimestamp;
    private static String TAG = "BaseClass";

    public SampleMecanumDriveBase() {
        super(DriveConstantsPID.kV, DriveConstantsPID.kA, DriveConstantsPID.kStatic, TRACK_WIDTH);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.txP, DriveConstantsPID.txI, DriveConstantsPID.txD);
        yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstantsPID.tyP, DriveConstantsPID.tyI, DriveConstantsPID.tyD);
        HEADING_PID = new PIDCoefficients(DriveConstantsPID.hP, DriveConstantsPID.hI, DriveConstantsPID.hD);

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(xTRANSLATIONAL_PID, yTRANSLATIONAL_PID, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();
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

        RobotLog.dd(TAG, "update: x " + currentPose.getX());
        RobotLog.dd(TAG, "y " + currentPose.getY());
        RobotLog.dd(TAG, "heading " + Double.toString(currentPose.getHeading()));

        RobotLog.dd(TAG, "xError " + lastError.getX());
        RobotLog.dd(TAG, "yError " + lastError.getY());
        RobotLog.dd(TAG, "headingError "  + lastError.getHeading());

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

        lastTimestamp = currentTimestamp;
        lastWheelPositions = positions;

        return velocities;
    }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);

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
}
