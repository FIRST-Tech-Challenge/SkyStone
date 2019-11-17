package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Basic: Autonomous League Test Drive", group="Linear Opmode")

public class AutonomousTestLeague extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftFront = null;   // white - port 2
    private DcMotor leftRear = null;    // yellow - port 1
    private DcMotor rightFront = null;  // green - port 3
    private DcMotor rightRear = null;   // blue - port 0

    private Servo clawServo = null;
    private CRServo armMotor = null;
    private DistanceSensor sensorRange = null;

    private DigitalChannel armStop = null;

    private double clawPos;
    private final double clawIncrement = 0.02;

    @Override
    public void runOpMode() throws InterruptedException {

        // Put initialization code between here and "waitforStart()"

        // Setup a variable for each side of the robot
        double leftPower;
        double rightPower;
        double targetRange;

        int rightFrontRange, leftFrontRange, rightRearRange, leftRearRange;

        final double MAX_RANGE = 36.0;
        final double MID_SPEED = 0.25;
        final double MID_RANGE = 12.0;
        final double MIN_SPEED = 0.125;
        final double MIN_RANGE = 1.0;

        final double TICKS_PER_ROT = 537.6;
        final double WHEEL_DIAMETER = 4;
        final double TICKS_PER_INCH = TICKS_PER_ROT/(Math.PI * WHEEL_DIAMETER);
        final double WHEEL_ANGLE = Math.PI/4;

        final double DRIVE_PWR = 0.25;

        // This driving mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        // strafing control

        double control = 2;

        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clawServo.setDirection(Servo.Direction.FORWARD);
        armMotor.setDirection(CRServo.Direction.FORWARD);

        // while the power of the wheels is 0, brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Claw position", "%4.2f", clawPos);

        waitForStart();

        // Put the autonomous portion of the code here

        // Move forward 3ft
        //    Determine the distance in number of ticks
        rightFrontRange = (int)(3*12*TICKS_PER_INCH/Math.cos(WHEEL_ANGLE)/2);

        leftFront.setTargetPosition(rightFrontRange);
        leftRear.setTargetPosition(rightFrontRange);
        rightFront.setTargetPosition(rightFrontRange);
        rightRear.setTargetPosition(rightFrontRange);

        leftFront.setPower(DRIVE_PWR);
        leftRear.setPower(DRIVE_PWR);
        rightFront.setPower(DRIVE_PWR);
        rightRear.setPower(DRIVE_PWR);

        while (leftFront.isBusy() || leftRear.isBusy() || rightFront.isBusy() || rightRear.isBusy()){
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

}
