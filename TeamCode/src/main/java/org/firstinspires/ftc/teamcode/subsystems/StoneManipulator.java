package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class StoneManipulator {
    private static StoneManipulator instance = null;
    private TouchSensor extenderForwardLimit;
    private TouchSensor extenderReverseLimit;
    private CRServo outtakeRight;
    private CRServo outtakeLeft;
    private Servo stoneGrabBig;
    private Servo stoneGrabLittle;
    private DcMotor rightIntake;
    private DcMotor leftIntake;
    private final double FORWARD_GRABBER_CLASPED_POSITION = .4;
    private final double FORWARD_GRABBER_UNCLASPED_POSITION = .9;
    private final double BACKWARD_GRABBER_CLASPED_POSITION = .4;
    private final double BACKWARD_GRABBER_UNCLASPED_POSITION = .9;
    private final double EXTEND_POWER = 1;
    private State currentState;
    public static synchronized StoneManipulator getInstance() {
        return instance != null ? instance : (instance = new StoneManipulator());
    }
    public void init(HardwareMap hardwareMap) {
        extenderForwardLimit = hardwareMap.get(TouchSensor.class, "extenderForwardLimit");
        extenderReverseLimit = hardwareMap.get(TouchSensor.class, "extenderReverseLimit");
        rightIntake = hardwareMap.get(DcMotor.class, "nubGrabberIntakeRight");
        leftIntake = hardwareMap.get(DcMotor.class, "nubGrabberIntakeLeft");
        stoneGrabBig = hardwareMap.get(Servo.class, "nubGrabBig");
        stoneGrabLittle = hardwareMap.get(Servo.class, "nubGrabLittle");
        outtakeRight = hardwareMap.get(CRServo.class, "outtakeRight");
        outtakeLeft = hardwareMap.get(CRServo.class, "outtakeLeft");
        stoneGrabBig.scaleRange(FORWARD_GRABBER_UNCLASPED_POSITION,FORWARD_GRABBER_CLASPED_POSITION);
        stoneGrabLittle.scaleRange(BACKWARD_GRABBER_CLASPED_POSITION,BACKWARD_GRABBER_UNCLASPED_POSITION);
        stoneGrabBig.setPosition(0);
        stoneGrabLittle.setPosition(0);
        leftIntake.setDirection(DcMotor.Direction.REVERSE); }
    public State setIntake (double power) {
        power = (currentState==State.CLASPING) ? 0 : power;
        leftIntake.setPower(power);
        rightIntake.setPower(power);
        currentState = (power != 0) ? State.INTAKEMOVING : (power==0) ? State.INTAKESTOPPED : State.INTAKESTOPPED;
        return currentState;
    } public State setExtended (boolean extended) {
        double power = EXTEND_POWER;
        power = (extended = false) ? -power : (currentState==State.UNCLASPED) ? 0 : power;
        outtakeRight.setPower(power);
        outtakeLeft.setPower(power);
        while (power != 0) {
            outtakeRight.setPower (extenderForwardLimit.isPressed() ? 0 : extenderReverseLimit.isPressed() ? 0 : power);
            outtakeLeft.setPower (extenderForwardLimit.isPressed() ? 0 : extenderReverseLimit.isPressed() ? 0 : power);
        }
        currentState = (extended = false) ? State.SLIDEMOVING : State.SLIDESTOPPED;
        return currentState;
    } public State setGrabbed (boolean isGrabbed) {
        double positionArmBig = (isGrabbed = true) ? FORWARD_GRABBER_CLASPED_POSITION : FORWARD_GRABBER_UNCLASPED_POSITION;
        double positionArmLittle = (isGrabbed = true) ? BACKWARD_GRABBER_CLASPED_POSITION : BACKWARD_GRABBER_UNCLASPED_POSITION;
        stoneGrabBig.setPosition(positionArmBig);
        stoneGrabLittle.setPosition(positionArmLittle);
        currentState = (isGrabbed = true) ? State.CLASPING : State.UNCLASPED;
        return currentState;
    } public State getState() {
        return currentState;
    }
    public enum State {
        INTAKEMOVING,
        INTAKESTOPPED,
        SLIDEMOVING,
        SLIDESTOPPED,
        CLASPING,
        UNCLASPED,
        RESTING,
        EXTENDED,
        INTAKING,


    }
}
