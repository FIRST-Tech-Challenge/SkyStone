package loki;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "loki.Spectrum")
public class Spectrum extends OpMode{
    //Defines Your Motors inside the code
    DcMotor lf, rf, rb, lb;

    //Encoder Setup
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double GEAR_DIAMETER_INCHES = 1.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (GEAR_DIAMETER_INCHES * 3.1415926535897932384626433);
    //Ticks are encoder units



    @Override
    public void init() {
        //Define your motors inside your phone.
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        //Motor name = hardwareMap.dcMotor.get("")

        //reverse right motors so it can not spin
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        double leftStickPower = gamepad1.left_stick_y;
        double rightStickPower = gamepad1.right_stick_x;

        lf.setPower(leftStickPower + rightStickPower);
        rf.setPower(-rightStickPower + leftStickPower);
        rb.setPower(-rightStickPower + leftStickPower);
        lb.setPower(leftStickPower + rightStickPower);


    }
}
