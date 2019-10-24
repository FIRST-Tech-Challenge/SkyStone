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

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Runs based on speed instead of voltage; makes run more consistently

        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    @Override
    public void loop() {
        //lf.setPower(0);
        //rf.setPower((0));
        //rb.setPower((0));
        //lb.setPower((0));

       /*Trig
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //returns hypotonuse (C value in triangle)

        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) + Math.PI / 4;
        //return angle x (next to center of circle)

        double rightX = gamepad1.right_stick_x; //reverses rotation
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
        //*/
        //gives wheels wheel power

        /*telemetry.addData("GamepadRx", gamepad1.right_stick_x);
        telemetry.addData("GamepadRy", gamepad1.right_stick_y);
        telemetry.addData("GamepadLy", gamepad1.left_stick_y);*/

        //No Trig
        /*
        double drive;
        double strafe;
        double rotate;
        double lfPow;
        double rfPow;
        double lbPow;
        double rbPow;

        drive = -gamepad1.left_stick_y;

        //strafe = gamepad1.left_stick_x; //add negative
        rotate = gamepad1.right_stick_x * 0.5;

        lfPow = drive + strafe + rotate;
        lbPow = drive - strafe + rotate;
        rfPow = drive - strafe - rotate;
        rbPow = drive + strafe - rotate;


        lf.setPower(lfPow);
        rf.setPower((rfPow));
        rb.setPower((rbPow));
        lb.setPower((lbPow));


       telemetry.addData("GamepadRx", gamepad1.right_stick_x);
        telemetry.addData("GamepadRy", gamepad1.right_stick_y);
        telemetry.addData("GamepadLy", gamepad1.left_stick_y);

        telemetry.addData("rb", lbPow);
        telemetry.addData("rf", rbPow);
        telemetry.addData("lf", lfPow);
        telemetry.addData("lb", lbPow);
        telemetry.update();//*/

        double leftPower;
        double rightPower;

        leftPower = Math.abs(gamepad1.left_stick_y) > 0.05? gamepad1.left_stick_y : 0;
        rightPower = Math.abs(gamepad1.right_stick_y) > 0.05? gamepad1.right_stick_y : 0;



        if(gamepad1.right_bumper){
            lf.setPower(-1);
            lb.setPower(1);
            rb.setPower(-1);
            rf.setPower(1);
        } else if(gamepad1.left_bumper){
            lf.setPower(1);
            lb.setPower(-1);
            rb.setPower(1);
            rf.setPower(-1);
        } else {
            lf.setPower(leftPower);
            lb.setPower(leftPower);
            rb.setPower(rightPower);
            rf.setPower(rightPower);
        }

        //Move Depot Hooks
        if (gamepad2.a) {
            clawR.setPosition(1);
            clawL.setPosition(1);
        }
        if (gamepad2.b) {
            clawR.setPosition(1);
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

