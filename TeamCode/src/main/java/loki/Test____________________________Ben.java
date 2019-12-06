package loki;

//imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="Test____________________________Ben")

public class Test____________________________Ben extends OpMode {
    //Define motors in code

 //   HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
 public DcMotor lf   = null;
    public DcMotor  rf  = null;
    public DcMotor lb   = null;
    public DcMotor  rb  = null;

    //Encoder Stuff
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double GEAR_DIAMETER_INCHES = 1.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (GEAR_DIAMETER_INCHES * 3.1415926535897932384626433);




    @Override
    public void init() {
        //Define Motors In Phone
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");

        //Reverse motors
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
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

            double drive;
            double strafe;
            double rotate;

            drive = -gamepad1.right_stick_y;
            strafe = gamepad1.right_stick_x;
            rotate = gamepad1.left_stick_x * 0.5;

            lf.setPower(drive + strafe + rotate);
            rf.setPower(drive - strafe + rotate);
            rb.setPower(drive - strafe - rotate);
            lb.setPower(drive + strafe - rotate);

            telemetry.addData("LB: ", lb.getCurrentPosition());
            telemetry.addData("LF: ", lf.getCurrentPosition());
            telemetry.addData("RB: ", rb.getCurrentPosition());
            telemetry.addData("RF: ", rf.getCurrentPosition());

            if (gamepad1.right_bumper) {
                lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }



        }
    }



