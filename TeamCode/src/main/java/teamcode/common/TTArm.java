package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TTArm {

    private static final double ELBOW_DEGREES_TO_TICKS = 3.506493506;
    private double MINIMUM_LIFT_DISTANCE_FROM_GROUND = 14;
    private double MAXIMUM_LIFT_INCHES_FROM_GROUND = 25;

    private final DcMotor lift, elbow, intake;
    private final DistanceSensor liftSensor;
    private final Servo claw;

    public TTArm(DcMotor lift, DistanceSensor liftSensor, DcMotor elbow, DcMotor intake, Servo claw) {
        this.lift = lift;
        this.liftSensor = liftSensor;
        this.elbow = elbow;
        this.intake = intake;
        this.claw = claw;
    }

    /**
     * @param inches how far above minimum position
     */
    public void setLiftHeight(double inches, double power) {
        if (inches < 0.0) {
            inches = 0.0;
        } else if (inches > MAXIMUM_LIFT_INCHES_FROM_GROUND - MINIMUM_LIFT_DISTANCE_FROM_GROUND) {
            inches = MAXIMUM_LIFT_INCHES_FROM_GROUND - MINIMUM_LIFT_DISTANCE_FROM_GROUND;
        }
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double startingHeight = getLiftHeight();
        if (inches < startingHeight) {
            lift.setPower(-Math.abs(power));
            while (getLiftHeight() > inches && TTOpMode.getOpMode().opModeIsActive()) {
                TTOpMode.getOpMode().telemetry.addData("current height", getLiftHeight());
                TTOpMode.getOpMode().telemetry.update();
            }
        } else if (inches > startingHeight) {
            lift.setPower(Math.abs(power));
            while (getLiftHeight() < inches && TTOpMode.getOpMode().opModeIsActive()) {
                TTOpMode.getOpMode().telemetry.addData("current height", getLiftHeight());
                TTOpMode.getOpMode().telemetry.update();
            }
        }
        lift.setPower(0.0);
    }

    public void liftContinuous(double power) {
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(power);
    }

    /**
     * Returns the number of inches that the lift is above its minimum position.
     */
    public double getLiftHeight() {
        return liftSensor.getDistance(DistanceUnit.INCH) - MINIMUM_LIFT_DISTANCE_FROM_GROUND;
    }

    public void rotate(int degrees, double power) {
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticks = (int) (degrees * ELBOW_DEGREES_TO_TICKS);
        elbow.setTargetPosition(ticks);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(power);
        while (elbow.isBusy()) ;
    }

    public void rotateContinuous(double power) {
        elbow.setPower(power);
    }

    public void intake(double power) {
        intake.setPower(power);
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    public double getClawPosition() {
        return claw.getPosition();
    }

}
