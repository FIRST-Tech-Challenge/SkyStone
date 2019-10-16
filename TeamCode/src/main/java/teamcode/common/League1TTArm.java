package teamcode.common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class League1TTArm {

    /**
     * In inches.
     */
    private static final double LIFT_HEIGHT_ERROR_TOLERANCE = 0.25;
    /**
     * In inches.
     */
    private static final double MIN_LIFT_HEIGHT = 4.0;
    /**
     * In inches.
     */
    private static final double MAX_LIFT_HEIGHT = 10.0;

    private static final double CLAW_OPEN_POS = 1.0;
    private static final double CLAW_CLOSE_POS = 0.0;

    private final CRServo lift;
    private final DistanceSensor liftSensor;
    private final Servo claw;

    public League1TTArm(HardwareMap hardwareMap) {
        lift = hardwareMap.get(CRServo.class, TTHardwareComponentNames.ARM_LIFT);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        liftSensor = hardwareMap.get(DistanceSensor.class, TTHardwareComponentNames.ARM_LIFT_SENSOR);
        claw = hardwareMap.get(Servo.class, TTHardwareComponentNames.ARM_CLAW);
    }

    public void setLiftHeight(double inches, double speed) {
        while (!liftNearTarget(inches)) {
            int sign;
            if (getLiftHeight() <= inches) {
                sign = 1;
            } else {
                sign = -1;
            }
            double power = sign * speed;
            lift.setPower(power);
        }
        lift.setPower(0.0);
    }

    private boolean liftNearTarget(double targetInches) {
        return Math.abs(getLiftHeight() - targetInches) <= LIFT_HEIGHT_ERROR_TOLERANCE;
    }

    public void liftContinuous(double power) {
        lift.setPower(power);
    }

    public double getLiftHeight() {
        return liftSensor.getDistance(DistanceUnit.INCH);
    }

    public void openClaw() {
        claw.setPosition(CLAW_OPEN_POS);
    }

    public void closeClaw() {
        claw.setPosition(CLAW_CLOSE_POS);
    }

    public boolean clawIsOpen() {
        return claw.getPosition() == CLAW_OPEN_POS;
    }

}
