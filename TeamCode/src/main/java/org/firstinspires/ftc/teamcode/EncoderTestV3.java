package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 * This encoder test program serves as a form of data collection for the tuning
 * of the P-Controller that is under creation at the moment. Please do not tamper with
 * the values of this program unless given permission to.
 *
 */


@Autonomous(name = "encoderTestv4", group = "Experimental")
public class EncoderTestV3 extends LinearOpMode {

    public int TPR = 1120;
    public int gearRatio = 1;
    public double circumference = 4 * 3.1459;
    public double unitRate = TPR / circumference;
    public DcMotor BL;
    public DcMotor TR;
    //TypexChart chart = new TypexChart();
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        BL = hardwareMap.get(DcMotor.class, "BL");
        //TR = hardwareMap.get(DcMotor.class, "TR");
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //chart.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            //Telemetry data is added to the DriverController screen
            telemetry.addData("Encoder", BL.getCurrentPosition());
            telemetry.addData("Past", toleranceStoppage(BL, targetPos(10), 27.3));

            //Used for repositioning the wheel
            BL.setPower(gamepad1.left_stick_y);

            //While loop that runs so long as it is not within the range of accepted values
            while ((toleranceStoppage(BL, targetPos(10), 26.7)) && gamepad1.a) {
                BL.setPower(0.1);
                telemetry.update();
            }
        }
    }

    /*
    The targetPos is slightly redundant, but it esentially allows for the user to input a certain
    distance as opposed to using pure encoder values. In other words, it takes in a certain amount
    in human distance (inches, centimeter, etc) and changes it to a value that the program can
    understand and use (encoder values).
     */

    public int targetPos(double dist) {
        int position = (int) Math.round(unitRate * dist);
        if (0 == 0) {
            return position;
        } else {
            return 0;
        }
    }

    /*
    This toleranceStoppage method works exactly like the threshold function that has been described
    in the Basic Auto file, but the main difference is that this function takes in a tolerance value
    that *should* take into account the relationship between the momentum and encoder error. The way
    that it works is by stopping by the amount that the error had calculated to be the difference
    in encoder distance.
    */

    public boolean toleranceStoppage(DcMotor wheel, int targetPos, double tolerance) {
        int intTolerance = (int) Math.round(tolerance);
        return wheel.getCurrentPosition() <= targetPos-intTolerance;
    }
}
