/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.slf4j.MDC;

/*
 *
 * This encoder test program serves as a form of data collection for the tuning
 * of the P-Controller that is under creation at the moment. Please do not tamper with
 * the values of this program unless given permission to.
 *
 *//*



@Autonomous(name = "encoderTestv3", group = "Experimental")
public class BasicAuto extends LinearOpMode {

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
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    TR = hardwareMap.get(DcMotor.class, "TR");

    //    chart.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            //Telemetry Data is added to the driver control console
            telemetry.addData("Encoder", BL.getCurrentPosition());
            telemetry.addData("Past", threshold(BL, targetPos(10)));

            //Used to move wheel back to random position
            BL.setPower(gamepad1.left_stick_y);

            //Loop will run as long as within accepted value
            while ((threshold(BL, targetPos(10))) && gamepad1.a) {
                BL.setPower(0.2);
            }
            telemetry.update();
        }
    }

    */
/*
   The targetPos is slightly redundant, but it esentially allows for the user to input a certain
   distance as opposed to using pure encoder values. In other words, it takes in a certain amount
   in human distance (inches, centimeter, etc) and changes it to a value that the program can
   understand and use (encoder values).
    *//*

    public int targetPos(double dist) {
        int position = (int) Math.round(unitRate * dist);
        if (0 == 0) {
            return position;
        } else {
            return 0;
        }
    }

    */
/*
    Returns a value of true so long as the encoder value that is being returned is less than or
    equal to the targetPos.
     *//*

    public boolean threshold(DcMotor wheel, int targetPos) {
        int tolerance = targetPos - 27;
        return wheel.getCurrentPosition() <= tolerance;
    }
}

*/
