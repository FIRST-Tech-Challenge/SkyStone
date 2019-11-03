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
            double rotRadCcw = 1 / solved[2];
            myLocation.direction = new Angle(
                    myLocation.direction.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING) - rotRadCcw,
                    Angle.AngleUnit.RADIANS,
                    Angle.AngleOrientation.COMPASS_HEADING
            );
            myLocation.direction.getValue(Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING);
            double diffx = -rotCenterRelX;
            double diffy = -rotCenterRelY;
            double atan = Math.atan2(diffy, diffx);
            atan += rotRadCcw;
            double radius = Math.hypot(diffx, diffy);
            double ndiffx = radius * Math.cos(atan);
            double ndiffy = radius * Math.sin(atan);
            myLocation.translate(ndiffx - diffx, ndiffy - diffy);
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
            int numWheels = 3;
            for (int i = 0; i < numWheels; i++) {
                double pivot = augmentedMatrix[i][i];
                for (int j = i + 1; j < numWheels; j++) {
                    double scalar = -pivot/augmentedMatrix[j][i];
                    if (scalar == 0 || augmentedMatrix[j][i] == 0) {
                        break;
                    }
                    for (int k = 0; k <= numWheels; k++) {
                        augmentedMatrix[j][k] *= scalar;
                    }
                    for (int k = 0; k <= numWheels; k++) {
                        augmentedMatrix[j][k] += augmentedMatrix[i][k];
                    }
                    augmentedMatrix[j][i] = 0;
                }
                if (pivot == 0) {
                    continue;
                }
                for (int k = 0; k <= numWheels; k++) {
                    augmentedMatrix[i][k] /= pivot;
                }
            }
            for (int i = numWheels - 1; i >= 0; i--) {
                for (int j = 0; j < i; j++) {
                    double[] newRow = new double[numWheels + 1];
                    for (int k = 0; k <= numWheels; k++) {
                        newRow[k] = augmentedMatrix[j][i]*augmentedMatrix[i][k];
                    }
                    for (int k = 0; k <= numWheels; k++) {
                        augmentedMatrix[j][k] -= newRow[k];
                    }
                    augmentedMatrix[j][i] = 0;
                }
            }
            double[] solutions = new double[numWheels];
            for (int k = 0; k < numWheels; k++) {
                solutions[k] = augmentedMatrix[k][numWheels];
            }
            return solutions;
        }
    }
}
