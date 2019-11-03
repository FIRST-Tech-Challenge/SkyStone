package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.ButtonAndEncoderData;
import org.firstinspires.ftc.teamcode.lib.MecanumDriveImpl;
import org.westtorrancerobotics.lib.Angle;
import org.westtorrancerobotics.lib.Location;
import org.westtorrancerobotics.lib.MecanumController;
import org.westtorrancerobotics.lib.MecanumDrive;

public class DriveTrain {

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private MecanumController mecanumController;

    private ColorSensor lineSpotter;
    private static final int RED_THRESHOLD  = 5;
    private static final int BLUE_THRESHOLD = 5;

    private Odometer odometer;

    private static DriveTrain instance = null;

    public static synchronized DriveTrain getInstance() {
        return instance != null ? instance : (instance = new DriveTrain());
    }

    private DriveTrain() {}

    public void init(HardwareMap hardwareMap) {
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lineSpotter = hardwareMap.get(ColorSensor.class, "lineColor");

        MecanumDrive wheels = new MecanumDriveImpl(leftFront, leftBack, rightFront, rightBack, null);
        mecanumController = new MecanumController(wheels);

        odometer = Odometer.getInstance();
        odometer.init(hardwareMap);
    }

    public void setMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        leftFront.setZeroPowerBehavior(mode);
        leftBack.setZeroPowerBehavior(mode);
        rightFront.setZeroPowerBehavior(mode);
        rightBack.setZeroPowerBehavior(mode);
    }

    public void spinDrive(double x, double y, double turn) {
        mecanumController.spinDrive(x, y, turn, MecanumDrive.TranslTurnMethod.EQUAL_SPEED_RATIOS);
    }

    public void updateLocation() {
        odometer.update();
    }

    public void setLocationZero() {
        odometer.myLocation = new Location(0, 0, new Angle(0, 1));
    }

    public void setLocation(Location l) {
        odometer.myLocation = l;
    }

    public Location getLocation() {
        return odometer.myLocation;
    }

    public boolean onRedLine() {
        return lineSpotter.red() > RED_THRESHOLD;
    }

    public boolean onBlueLine() {
        return lineSpotter.blue() > BLUE_THRESHOLD;
    }

    private static class Odometer {
        private Location myLocation;
        private Wheel leftY;
        private Wheel rightY;
        private Wheel x;

        private final double TICKS_TO_INCHES = (2 * Math.PI) / 4096;

        private static Odometer instance = null;

        public static synchronized Odometer getInstance() {
            return instance != null ? instance : (instance = new Odometer());
        }

        private Odometer() {}

        public void init(HardwareMap hardwareMap) {
            myLocation = new Location(0, 0, new Angle(0, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING));
            leftY = new Wheel(new Location(-6.815,1.645,
                    new Angle(180, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING)),
                    hardwareMap.get(DcMotorEx.class, "intakeLeft/odometerLeftY"));
            rightY = new Wheel(new Location(6.815,1.645,
                    new Angle(0, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING)),
                    hardwareMap.get(DcMotorEx.class, "intakeRight/odometerRightY"));
            x = new Wheel(new Location(7.087,-1.980,
                    new Angle(-90, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING)),
                    hardwareMap.get(DcMotorEx.class, "liftRight/odometerX"));
        }

        public void update() {
            if ((ButtonAndEncoderData.getLatest().getCurrentPosition(leftY.encoder) - leftY.lastEnc) ==
                    (ButtonAndEncoderData.getLatest().getCurrentPosition(rightY.encoder) - rightY.lastEnc)) {
                long dy = (ButtonAndEncoderData.getLatest().getCurrentPosition(leftY.encoder) - leftY.lastEnc);
                long dx = (ButtonAndEncoderData.getLatest().getCurrentPosition(x.encoder) - x.lastEnc);
                leftY.lastEnc += dy;
                rightY.lastEnc += dy;
                x.lastEnc += dx;
                myLocation.translate(dx * TICKS_TO_INCHES, dy * TICKS_TO_INCHES);
                return;
            }
            double[] solved = solve(new double[][]{
                    {
                            Math.cos(leftY.fetchDirection()),
                            -Math.sin(leftY.fetchDirection()),
                            -leftY.getMovedInches(),
                            Math.cos(leftY.fetchDirection()) * leftY.relativeLocation.x
                                    - Math.sin(leftY.fetchDirection()) * leftY.relativeLocation.y
                    },
                    {
                            Math.cos(rightY.fetchDirection()),
                            -Math.sin(rightY.fetchDirection()),
                            -rightY.getMovedInches(),
                            Math.cos(rightY.fetchDirection()) * rightY.relativeLocation.x
                                    - Math.sin(rightY.fetchDirection()) * rightY.relativeLocation.y
                    },
                    {
                            Math.cos(x.fetchDirection()),
                            -Math.sin(x.fetchDirection()),
                            -x.getMovedInches(),
                            Math.cos(x.fetchDirection()) * x.relativeLocation.x
                                    - Math.sin(x.fetchDirection()) * x.relativeLocation.y
                    },
                }
            );
            double rotCenterRelX = solved[0];
            double rotCenterRelY = solved[1];
            double rotRadCw = 1 / solved[2];
            myLocation.direction = new Angle(
                    myLocation.direction.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING) + rotRadCw,
                    Angle.AngleUnit.RADIANS,
                    Angle.AngleOrientation.COMPASS_HEADING
            );
            double oldX = myLocation.x;
            double oldY = myLocation.y;
            double r = Math.hypot(rotCenterRelX, rotCenterRelY);
            double hr = rotCenterRelX + oldX;
            double kr = rotCenterRelY + oldY;
            double theta = -rotRadCw + Math.atan2(-rotCenterRelY, -rotCenterRelX);
            myLocation.setLocation(hr + r * Math.cos(theta), kr + r * Math.sin(theta));
        }

        private class Wheel {
            private final Location relativeLocation;
            private final double direction;
            private final DcMotorEx encoder;
            private long lastEnc;

            private Wheel (Location relativeLocation, DcMotorEx encoder) {
                this.relativeLocation = relativeLocation;
                this.encoder = encoder;
                lastEnc = ButtonAndEncoderData.getLatest().getCurrentPosition(encoder);
                direction = relativeLocation.direction.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING);
            }

            double getMovedInches() {
                long dx = ButtonAndEncoderData.getLatest().getCurrentPosition(encoder) - lastEnc;
                lastEnc += dx;
                return dx * TICKS_TO_INCHES;
            }

            double fetchDirection() {
                return direction;
            }
        }

        private double[] solve(double[][] augmentedMatrix) {
            double[][] matrix = new double[augmentedMatrix.length][augmentedMatrix.length + 1];
            for (int i = 0; i < matrix.length; i++) {
                System.arraycopy(augmentedMatrix[i], 0, matrix[i], 0, matrix[i].length);
            }
            for (int i = 0; i < matrix.length; i++) {
                for (int j = i; j < matrix.length; j++) {
                    if (!isZero(matrix[j][i])) {
                        swapRows(matrix, i, j);
                        break;
                    }
                }
                scale(matrix, i, 1 / matrix[i][i]);
                matrix[i][i] = 1;
                for (int j = i + 1; j < matrix.length; j++) {
                    if (isZero(matrix[j][i])) {
                        continue;
                    }
                    scale(matrix, j, -1 / matrix[j][i]);
                    addRow(matrix, i, j);
                    matrix[j][i] = 0;
                }
            }
            for (int i = matrix.length - 1; i >= 0; i--) {
                for (int j = i - 1; j >= 0; j--) {
                    if (isZero(matrix[j][i])) {
                        continue;
                    }
                    double sc = -matrix[j][i];
                    scale(matrix, i, sc);
                    addRow(matrix, i, j);
                    scale(matrix, i, 1/sc);
                }
            }
            double[] solutions = new double[matrix.length];
            for (int k = 0; k < matrix.length; k++) {
                solutions[k] = matrix[k][matrix.length];
            }
            return solutions;
        }

        private boolean isZero(double number) {
            return Math.abs(number) < 1e-15;
        }

        private void swapRows(double[][] matrix, int row1, int row2) {
            double[] rowa = matrix[row2];
            matrix[row2] = matrix[row1];
            matrix[row1] = rowa;
        }

        private void scale(double[][] matrix, int row, double constant) {
            for (int i = 0; i < matrix[row].length; i++) {
                matrix[row][i] *= constant;
            }
        }

        private void addRow(double[][] matrix, int addend, int rowChanged) {
            for (int i = 0; i < matrix[rowChanged].length; i++) {
                matrix[rowChanged][i] += matrix[addend][i];
            }
        }
    }
}
