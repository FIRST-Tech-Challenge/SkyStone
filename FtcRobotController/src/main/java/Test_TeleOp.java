import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Test_TeleOp")
public class Test_TeleOp extends OpMode{
    DcMotor lf, rf, lb, rb, ls; //Define Motors In Code
    public Gamepad g1, g2;
    Servo clawL, clawR;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        //Motor Define In Phone
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        ls = hardwareMap.dcMotor.get("ls");
        //Servo Define
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

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

        strafe = gamepad1.left_stick_x; //add negative
        rotate = gamepad1.right_stick_x * 0.5;

        lfPow = drive + strafe + rotate;
        lbPow = drive - strafe + rotate;
        rfPow = drive - strafe - rotate;
        rbPow = drive + strafe - rotate;


        lf.setPower(lfPow);
        rf.setPower((rfPow));
        rb.setPower((rbPow));
        lb.setPower((lbPow));

       /* telemetry.addData("GamepadRx", gamepad1.right_stick_x);
        telemetry.addData("GamepadRy", gamepad1.right_stick_y);
        telemetry.addData("GamepadLy", gamepad1.left_stick_y); */

        telemetry.addData("rb", lbPow);
        telemetry.addData("rf", rbPow);
        telemetry.addData("lf", lfPow);
        telemetry.addData("lb", lbPow);

        //Move Depot Hooks
        if (gamepad2.a) {
            clawR.setPosition(1);
            clawL.setPosition(0);
        }
        if (gamepad2.b) {
            clawR.setPosition(0);
            clawL.setPosition(1);
        }

        //Linear Slide
        ls.setPower(gamepad2.left_stick_y);


    }

  /*  public class JoystickCalc
    {
        private OpMode opmode;

        double leftStickY;
        double leftStickX;
        double rightStickX;
        double rightStickY;
        boolean xButton;
        boolean yButton;
        boolean aButton;
        boolean bButton;

        public JoystickCalc(OpMode opmode)
        {
            this.opmode = opmode;
        }

        public void calculate ()
        {
            leftStickY = opmode.gamepad1.left_stick_y;
            leftStickX = opmode.gamepad1.left_stick_x;
            rightStickX = opmode.gamepad1.right_stick_x;
            rightStickY = opmode.gamepad1.right_stick_y;
            xButton = opmode.gamepad1.x;
            yButton = opmode.gamepad1.y;
            bButton = opmode.gamepad1.b;
            aButton = opmode.gamepad1.a;
        }*/
    }

