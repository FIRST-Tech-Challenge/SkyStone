package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotlib.drivetrain.EncoderMotor;
import org.firstinspires.ftc.robotlib.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.robotlib.managers.MotorManager;
import org.firstinspires.ftc.robotlib.managers.ServoManager;
import org.firstinspires.ftc.robotlib.servo.CompelledServo;
import org.firstinspires.ftc.robotlib.servo.ModifiedServo;
import org.firstinspires.ftc.robotlib.state.ServoState;

import static org.firstinspires.ftc.robotlib.state.ServoState.CARRY;
import static org.firstinspires.ftc.robotlib.state.ServoState.CRADLE;
import static org.firstinspires.ftc.robotlib.state.ServoState.FLOOR;
import static org.firstinspires.ftc.robotlib.state.ServoState.ONEBLOCKDEPOSIT;
import static org.firstinspires.ftc.robotlib.state.ServoState.THREEBLOCKDEPOSIT;
import static org.firstinspires.ftc.robotlib.state.ServoState.THREEBLOCKHOVER;
import static org.firstinspires.ftc.robotlib.state.ServoState.TWOBLOCKDEPOSIT;
import static org.firstinspires.ftc.robotlib.state.ServoState.TWOBLOCKHOVER;

/**
 * Hardware map with extended functionality using the FTC provided HardwareMap.
 */
public class MecanumHardwareMap {
    public HardwareMap internalHardwareMap;

    // Motors in Mecanum robot
    private EncoderMotor driveFrontLeft;
    private EncoderMotor driveFrontRight;
    private EncoderMotor driveRearRight;
    private EncoderMotor driveRearLeft;

    // Intake
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    // Servos
    public ModifiedServo blockGrabber;
    public CompelledServo deliveryLeft;
    public CompelledServo deliveryRight;
    private ServoState[] deliveryStates = new ServoState[]{CRADLE, CARRY, TWOBLOCKHOVER, TWOBLOCKDEPOSIT, ONEBLOCKDEPOSIT, FLOOR};
    private ServoState[] deliveryStatesThreeBlock = new ServoState[]{CRADLE, CARRY, THREEBLOCKHOVER, THREEBLOCKDEPOSIT, TWOBLOCKHOVER, TWOBLOCKDEPOSIT, ONEBLOCKDEPOSIT, FLOOR};

    public Servo platformServoLeft;
    public Servo platformServoRight;

    // Camera
    public WebcamName webcamName;

    public MecanumDrivetrain drivetrain;

    // Managers (for treating a group of objects as one)
    //public ServoManager deliveryServoManager;
    public MotorManager intakeMotorManager;

    public final double wheelRadius = 4; //inches
    private static final double wheelToMotorRatio = 1.0/1.0;

    public EncoderMotor[] motorList;

    public final double motorTicksPerInch;

    /**
     * Creates a mecanum hardware map from the FTC given hardware map.
     * @param hwMap FTC hardware map
     */
    public MecanumHardwareMap(HardwareMap hwMap) {
        this.internalHardwareMap = hwMap;

        driveFrontLeft = new EncoderMotor(hwMap.get(DcMotor.class, "driveFrontLeft"));
        driveFrontRight = new EncoderMotor(hwMap.get(DcMotor.class, "driveFrontRight"));
        driveRearRight = new EncoderMotor(hwMap.get(DcMotor.class, "driveRearRight"));
        driveRearLeft = new EncoderMotor(hwMap.get(DcMotor.class, "driveRearLeft"));

        intakeLeft = hwMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hwMap.get(DcMotor.class, "intakeRight");

        blockGrabber = new ModifiedServo(hwMap.get(Servo.class, "blockGrabber"));
        deliveryLeft = new CompelledServo(hwMap.get(Servo.class, "deliveryLeft"), deliveryStatesThreeBlock);
        deliveryRight = new CompelledServo(hwMap.get(Servo.class, "deliveryRight"), deliveryStatesThreeBlock);
        deliveryRight.setOffset(0.04);

        platformServoLeft = hwMap.get(Servo.class, "platformServoLeft");
        platformServoRight = hwMap.get(Servo.class, "platformServoRight");

        motorList = new EncoderMotor[]{driveFrontLeft, driveFrontRight, driveRearLeft, driveRearRight};

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRearLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        blockGrabber.setDirection(Servo.Direction.FORWARD);
        deliveryLeft.setDirection(Servo.Direction.FORWARD);
        deliveryRight.setDirection(Servo.Direction.REVERSE);

        webcamName = hwMap.get(WebcamName.class, "Webcam 1");

        drivetrain = new MecanumDrivetrain(motorList);
        intakeMotorManager = new MotorManager(new DcMotor[]{intakeLeft, intakeRight});
        //deliveryServoManager = new ServoManager(new Servo[] {deliveryLeft, deliveryRight});

        motorTicksPerInch = drivetrain.getTicksPerInch(wheelRadius, wheelToMotorRatio);
    }

    /**
     * Shortcut for updating the two delivery servos.
     * @param newState New Servo state (must be in the possible positions)
     */
    public void updateDeliveryStates(ServoState newState) {
        this.deliveryLeft.setState(newState);
        this.deliveryRight.setState(newState);
    }
}
