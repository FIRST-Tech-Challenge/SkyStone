package org.firstinspires.ftc.teamcode.alek_wang;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="4100Test-alwang", group="4100")
@Disabled
public class Mecanum4100 extends LinearOpMode {
    private DcMotor lf,lb = null;
    private DcMotor rf,rb = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double a,b,c,d;//wheels a=lf b=rf c=lb d=rb
            double lfpower,rfpower,lbpower,rbpower;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            a = gamepad1.left_stick_y;
            b = gamepad1.left_stick_x;
            c = gamepad1.left_trigger;
            d = gamepad2.right_trigger;
            lfpower = Range.clip(a+b-c+d, -1.0, 1.0);
            rfpower = Range.clip(a-b+c-d, -1.0, 1.0);
            lbpower = Range.clip(a-b-c+d, -1.0, 1.0);;
            rbpower = Range.clip(a+b+c-d, -1.0, 1.0);

            // Send calculated power to wheels
            lf.setPower(lfpower);
            rf.setPower(rfpower);
            lb.setPower(lbpower);
            rb.setPower(rbpower);
        }
    }
}
