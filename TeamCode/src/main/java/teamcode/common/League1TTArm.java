package teamcode.common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class League1TTArm {

    private static final int RED_THRESHOLD = 800;
    private static final int BLUE_THRESHOLD = 800;
    private static final double CLAW_OPEN_POS = 1.0;
    private static final double CLAW_CLOSE_POS = 0.0;

    private final CRServo lift;
    private final ColorSensor liftSensor;
    private final Servo claw;

    public League1TTArm(HardwareMap hardwareMap) {
        lift = hardwareMap.get(CRServo.class, TTHardwareComponentNames.ARM_LIFT);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        liftSensor = hardwareMap.get(ColorSensor.class, TTHardwareComponentNames.ARM_LIFT_SENSOR);
        claw = hardwareMap.get(Servo.class, TTHardwareComponentNames.ARM_CLAW);
    }

    public void testColorSensor(Telemetry telemetry) {
        int red = liftSensor.red();
        int blue = liftSensor.blue();
        telemetry.addData("red", red);
        telemetry.addData("blue", blue);
        LiftColor color = getColor();
        telemetry.addData("color detected", color);
        telemetry.update();
    }

    public void raise(double power) {
        power = Math.abs(power);
        while (getColor() != LiftColor.RED) {
            lift.setPower(power);
        }
        lift.setPower(0.0);
    }

    public void lower(double power) {
        power = Math.abs(power);
        while (getColor() != LiftColor.BLUE) {
            lift.setPower(-power);
        }
        lift.setPower(0.0);
    }

    private LiftColor getColor() {
        int r = liftSensor.red();
        int b = liftSensor.blue();
        if (r < 800 && b < 800) {
            if (r > b) {
                return LiftColor.RED;
            } else if (b > r) {
                return LiftColor.BLUE;
            }
        }
        return LiftColor.NONE;
    }

    public void liftContinuous(double power) {
        lift.setPower(power);
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

    private enum LiftColor {
        RED, BLUE, NONE
    }

}
