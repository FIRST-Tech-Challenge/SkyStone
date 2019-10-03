import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Telemetry")
public class Telemetry extends OpMode {
    DcMotor lf, rf, lb, rb;
    public Gamepad g1, g2;
    Servo clawL, clawR;
    private ElapsedTime runtime = new ElapsedTime();
    //Encoder Setup
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double GEAR_DIAMETER_INCHES = 1.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (GEAR_DIAMETER_INCHES * 3.1415926535897932384626433);


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        //Motor Define
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        //Servo Define
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        //Encoder Stuff
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);





    }

    @Override
    public void loop() {


        /* //Trig
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //returns hypotonuse (C value in triangle)

        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        //return angle x (next to center of circle)

        double rightX = -gamepad1.right_stick_x;
        //rotiation

        final double lfPow = r * Math.cos(robotAngle) - rightX;
        final double rfPow = r * Math.sin(robotAngle) + rightX;
        final double lbPow = r * Math.sin(robotAngle) - rightX;
        final double rbPow = r * Math.cos(robotAngle) + rightX;
        //determines wheel power

        rf.setPower(rfPow);
        rb.setPower(rbPow);
        lf.setPower(lfPow);
        lb.setPower(lbPow);
        //gives wheels wheel power

        telemetry.addData("GamepadRx", gamepad1.right_stick_x);
        telemetry.addData("GamepadRy", gamepad1.right_stick_y);
        telemetry.addData("GamepadLy", gamepad1.left_stick_y);

        telemetry.update(); */
        //No Trig
        double drive;
        double strafe;
        double rotate;
        double lfPow;
        double rfPow;
        double lbPow;
        double rbPow;

        drive = -gamepad1.left_stick_y;

        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x * 0.5;

        lfPow = drive + strafe + rotate;
        lbPow = drive - strafe + rotate;
        rfPow = drive - strafe - rotate;
        rbPow = drive + strafe - rotate;


        lf.setPower(lfPow);
        rf.setPower((rfPow));
        rb.setPower((rbPow));
        lb.setPower((lbPow));

        /*telemetry.addData("GamepadRx", gamepad1.right_stick_x);
        telemetry.addData("GamepadRy", gamepad1.right_stick_y);
        telemetry.addData("GamepadLy", gamepad1.left_stick_y);

        telemetry.addData("rb", lbPow);
        telemetry.addData("rf", rbPow);
        telemetry.addData("lf", lfPow);
        telemetry.addData("lb", lbPow);*/

        //Move Depot Hooks
        if (gamepad2.a) {
            clawR.setPosition(1);
            clawL.setPosition(0);
        }
        if (gamepad2.b) {
            clawR.setPosition(0);
            clawL.setPosition(1);
        }

        telemetry.addData("LB: ", lb.getCurrentPosition());
        telemetry.addData("LF: ", lf.getCurrentPosition());
        telemetry.addData("RB: ", rb.getCurrentPosition());
        telemetry.addData("RF: ", rf.getCurrentPosition());

        telemetry.update();

    }
}
