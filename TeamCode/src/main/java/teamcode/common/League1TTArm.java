package teamcode.common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Timer;
import java.util.TimerTask;

public class League1TTArm {

    private static final double CLAW_OPEN_POS = 0.5;
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
        telemetry.addData("lift power", lift.getPower());
        telemetry.update();
    }

    public void raise(double power) {
        power = Math.abs(power);
        final boolean[] stopLift = new boolean[1];
        scheduleStopLiftFlag(stopLift, 2);
        Telemetry telemetry = TTOpMode.currentOpMode().telemetry;
        while (getColor() != LiftColor.RED && !stopLift[0] && TTOpMode.currentOpMode().opModeIsActive()) {
            testColorSensor(telemetry);
            lift.setPower(power);
        }
        lift.setPower(0.0);
        testColorSensor(telemetry);
    }

    public void lower(double power) {
        power = Math.abs(power);
        final boolean[] stopLift = new boolean[1];
        scheduleStopLiftFlag(stopLift, 2);
        Telemetry telemetry = TTOpMode.currentOpMode().telemetry;
        while (getColor() != LiftColor.BLUE && !stopLift[0] && TTOpMode.currentOpMode().opModeIsActive()) {
            testColorSensor(telemetry);
            lift.setPower(-power);
        }
        lift.setPower(0.0);
        testColorSensor(telemetry);
    }

    public void liftTimed(double seconds, double power) {
        lift.setPower(power);
        boolean[] stopLift = new boolean[1];
        scheduleStopLiftFlag(stopLift, seconds);
        while (!stopLift[0] && TTOpMode.currentOpMode().opModeIsActive()) ;
        lift.setPower(0.0);
    }

    /**
     * Assigns true to the first element of the array once the timeout period has passed.
     */
    private void scheduleStopLiftFlag(final boolean[] stopLiftFlag, double seconds) {
        TimerTask stop = new TimerTask() {
            @Override
            public void run() {
                stopLiftFlag[0] = true;
            }
        };
        TTOpMode.currentOpMode().getTimer().schedule(stop, (long) (seconds * 1000));
    }

    private LiftColor getColor() {
        int r = liftSensor.red();
        int b = liftSensor.blue();
        if (r < 800 && b < 800 && r > 250 && b > 250) {
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
