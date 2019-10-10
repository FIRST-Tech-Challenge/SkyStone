package org.firstinspires.ftc.teamcode;

/*import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;*/
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Path 1 (TL/BR)", group = "AutoPaths")
public class Autonomous_Path1 extends LinearOpMode {


    private AutonomousFunctions AutoFunc = null;
    //  Declare hardware variables
    //Declare motors
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    // Detector object
   // private GoldAlignDetector detector = null;

    public void Initialise() {
        //Initialise hardware variables from configuration.
        //Phone configuration = "Testing_v2"
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        //Initialise AutonomousFunctions instance with hardware variables; set time interval
        AutoFunc = new AutonomousFunctions(leftMotor, rightMotor, 500);
        // Create detector, initialize it with the app context and camera
        //detector = new GoldAlignDetector();
       // AutoFunc.InitialiseDetector(detector, hardwareMap.appContext);
    }

    @Override
    public void runOpMode() {

        Initialise();

        sleep(4000);



           /* while (!detector.getAligned()) {
                double MoveToX = AutoFunc.DetermineMovementOffset(detector);
                telemetry.addData("MoveToX", MoveToX);
                telemetry.addData("X Pos", detector.getXPosition());
                telemetry.update();

                if (MoveToX > 0) {
                    leftMotor.setPower(0.75);
                    rightMotor.setPower(0.1);
                    sleep(500);
                } else if (MoveToX < 0) {
                    leftMotor.setPower(0.1);
                    rightMotor.setPower(0.75);
                    sleep(500);
                }
            }
            if (detector.getAligned()) {
                AutoFunc.MoveForwards(2000, 0.75);
            }
             */

    }


}
