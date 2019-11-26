package loki;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp (name = "loki.Trigonometry")
public class Trigonometry extends OpMode {
    DcMotor lf, rf, lb, rb;

    public Gamepad g1, g2;
    private ElapsedTime runtime = new ElapsedTime();





    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        //joystickCalc = new JoystickCalc(this);
    }

    @Override
    public void loop() {

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
    }

   /* public class JoystickCalc
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

            joystickCalc.calculate();
            test1 = joystickCalc.leftStickX;
            telemetry.addData("X", test1);
            telemetry.update();
        }
    }*/
}


