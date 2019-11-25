package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.common.math.TimePose;
import org.openftc.revextensions2.RevBulkData;

import java.util.LinkedList;

@Config
public class TwoWheelTrackingLocalizer {
    static final double TICKS_PER_REV = 4 * 600;
    public static double PARALLEL_WHEEL_RADIUS = 2.5;
    public static double LATERAL_WHEEL_RADIUS = 2.5;
    public static int VELOCITY_READ_TICKS = 5;

    static final double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_Y_POS = 7.25;
    public static double LATERAL_X_POS = 7.25;

    DecompositionSolver forwardSolver;

    int[] prevWheelPositions;
    double prevHeading;
    int[] wheelPorts;

    // External interfaces
    public Pose currentPosition;
    public Pose relativeRobotMovement;
    public LinkedList<TimePose> prevPositions = new LinkedList<>();

    public TwoWheelTrackingLocalizer(int parallelEncoder, int lateralEncoder) {
        this(parallelEncoder, lateralEncoder, new TimePose(new Pose(0, 0, 0)));
    }

    public TwoWheelTrackingLocalizer(int parallelEncoder, int lateralEncoder, TimePose start) {
        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);

        EncoderWheel[] WHEELS = {
                new EncoderWheel(0, PARALLEL_Y_POS, Math.toRadians(180), 0, 0), // parallel
                new EncoderWheel(LATERAL_X_POS, 0, Math.toRadians(90), 1, 1), // lateral
        };

        for (EncoderWheel wheelPosition : WHEELS) {
            double x = Math.cos(wheelPosition.heading);
            double y = Math.sin(wheelPosition.heading);

            inverseMatrix.setEntry(wheelPosition.row, 0, x);
            inverseMatrix.setEntry(wheelPosition.row, 1, y);
            inverseMatrix.setEntry(wheelPosition.row, 2,
                    wheelPosition.x * y - wheelPosition.y * x);
        }
        inverseMatrix.setEntry(2, 2, 1.0);

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();

        if (!forwardSolver.isNonSingular()) {
            throw new IllegalArgumentException("The specified configuration cannot support full localization");
        }

        prevWheelPositions = new int[2]; // Initializes with zeros
        wheelPorts = new int[]{parallelEncoder, lateralEncoder};

        currentPosition = new Pose(start.x, start.y, start.heading);
        relativeRobotMovement = new Pose(0, 0, 0);
        prevPositions.add(start);
    }

    public static double encoderTicksToCM(int ticks, double wheel_radius) {
        return wheel_radius * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static int cmToEncoderTicks(double cm) {
        return (int) Math.round(cm * TICKS_PER_REV / (PARALLEL_WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO));
    }

    public void update(RevBulkData data1, RevBulkData data2, double heading) {
        heading = (Math.PI / 180) * heading; //convert to radians
        double[] deltas = new double[] {
                encoderTicksToCM(data2.getMotorCurrentPosition(0) - prevWheelPositions[0],
                        PARALLEL_WHEEL_RADIUS),
                encoderTicksToCM(data1.getMotorCurrentPosition(0) - prevWheelPositions[1],
                        LATERAL_WHEEL_RADIUS),
                MathUtil.angleWrap(heading - prevHeading)
        };

        prevWheelPositions[0] = data2.getMotorCurrentPosition(0);
        prevHeading = heading;
        prevWheelPositions[1] = data1.getMotorCurrentPosition(0);

        RealMatrix m = MatrixUtils.createRealMatrix(new double[][] {deltas});

        RealMatrix rawPoseDelta = forwardSolver.solve(m.transpose());
        Pose robotPoseDelta = new Pose(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        );

        relativeRobotMovement = relativeRobotMovement.add(robotPoseDelta);
        currentPosition = MathUtil.relativeOdometryUpdate(currentPosition, robotPoseDelta);
        prevPositions.add(new TimePose(currentPosition));
    }

    public void virtualUpdate(TimePose t) {
        this.currentPosition = new Pose(t.x, t.y, t.heading);
        prevPositions.add(t);
    }

    public double x() { return currentPosition.x; }
    public double y() { return currentPosition.y; }
    public double h() { return currentPosition.heading; }
    public Pose pose() {
        return new Pose(currentPosition.x, currentPosition.y, currentPosition.heading);
    }

    public Pose velocity() {
        if (prevPositions.size() < 2) {
            return new Pose(0, 0, 0);
        }

        // We'd like to pick a time up to five reads ago, but we might not be able to
        int oldIndex = Math.max(0, prevPositions.size() - VELOCITY_READ_TICKS - 1);
        TimePose old = prevPositions.get(oldIndex);
        TimePose cur = prevPositions.get(prevPositions.size() - 1);

        double scaleFactor = (double) (cur.time - old.time) / (1000);
        return cur.minus((Pose) old).scale(1 / scaleFactor);
    }
}