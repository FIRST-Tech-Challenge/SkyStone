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
        telemetry.addData("lift power", lift.getPower());
        telemetry.update();
    }

    public void raise(double power) {
        power = Math.abs(power);
        Telemetry telemetry = TTOpMode.currentOpMode().telemetry;
        while (getColor() != LiftColor.RED && TTOpMode.currentOpMode().opModeIsActive()) {
            testColorSensor(telemetry);
            lift.setPower(power);
            stopLift(2);
        }
        lift.setPower(0.0);
        testColorSensor(telemetry);
    }
    public void lower(double power) {
        power = Math.abs(power);
        Telemetry telemetry = TTOpMode.currentOpMode().telemetry;
        while (getColor() != LiftColor.BLUE && TTOpMode.currentOpMode().opModeIsActive()) {
            testColorSensor(telemetry);
            lift.setPower(-power);
            stopLift(2);
        }
        lift.setPower(0.0);
        testColorSensor(telemetry);
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
    private void stopLift(double time) {
        Timer timer = TTOpMode.currentOpMode().getTimer();
        TimerTask stopMotor = new TimerTask() {
            @Override
            public void run() {
                lift.setPower(0.0);
            }
        };
        timer.schedule(stopMotor, (int) time * 1000);
    }

    public void timedLift(double seconds, double power){
        lift.setPower(power);
        stopLift(seconds);
    }

}
