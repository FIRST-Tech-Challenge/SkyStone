package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class StoneManipulator {
    private static StoneManipulator instance = null;
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
    private State currentState;
    public static synchronized StoneManipulator getInstance() {
        return instance != null ? instance : (instance = new StoneManipulator());
    }
    public void init(HardwareMap hardwareMap) {
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
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
    }
    public State setIntake (double power) {
        power = (currentState==State.CLASPING) ? 0 : power;
        leftIntake.setPower(power);
        rightIntake.setPower(power);
        currentState = (power>0 || power<0) ? State.INTAKEMOVING : (power==0) ? State.INTAKESTOPPED : State.INTAKESTOPPED;
        return currentState;
    } public State setOuttake (double power) {
        power = (currentState==State.UNCLASPED) ? 0 : power;
        outtakeRight.setPower(power);
        outtakeLeft.setPower(power);
        currentState = (power>0 || power<0) ? State.SLIDEMOVING : (power == 0) ? State.SLIDESTOPPED: State.SLIDESTOPPED;
        return currentState;
    } public State setStoneGrabber (double position) {
        stoneGrabBig.setPosition(position);
        stoneGrabLittle.setPosition(position);
        currentState = (stoneGrabLittle.getPosition()< position && stoneGrabBig.getPosition()< position) ? State.CLASPING : State.UNCLASPED;
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
    }
}
