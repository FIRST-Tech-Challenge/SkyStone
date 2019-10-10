package org.firstinspires.ftc.teamcode;

/*import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Path 1 (TL/BR) - Linear slide", group = "AutoPaths")
public class Autonomous_Path1_LinearSlide extends LinearOpMode {


    AutonomousFunctions AutoFunc = null;
    //  Declare hardware variables
    //Declare motors
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    // Detector object
    private GoldAlignDetector detector = null;
    private DcMotor slideMotor;
    private CRServo grabberServo;
    private CRServo intakeServo;
    private CRServo boxFlipServo;

    public void Initialise() {
        //Initialise hardware variables from configuration.
        //Phone configuration = "Testing_v2"
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        //Initialise AutonomousFunctions instance with hardware variables; set time interval
        AutoFunc = new AutonomousFunctions(leftMotor, rightMotor, 500);
        // Create detector, initialize it with the app context and camera
        detector = new GoldAlignDetector();
        AutoFunc.InitialiseDetector(detector, hardwareMap.appContext);

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        grabberServo = hardwareMap.get(CRServo.class, "grabberServo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        boxFlipServo = hardwareMap.get(CRServo.class, "boxFlipServo");

    }

    @Override
    public void runOpMode() {

        Initialise();

        sleep(4000);


        // Have a look at this autonomous code.
        intakeServo.setPower(0.9);
        sleep(1000);
        intakeServo.setPower(0);

        grabberServo.setPower(0.9);
        sleep(500);
        grabberServo.setPower(0);

        slideMotor.setPower(-1);
        sleep(2000);
        slideMotor.setPower(0);

        while (!detector.getAligned()) {
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
            AutoFunc.MoveForwards(1000, 0.75);
        }

        rightMotor.setPower(1);
        leftMotor.setPower(-1);
        sleep(1000);

        AutoFunc.MoveForwards(3000,0.9);

        rightMotor.setPower(0.3);
        leftMotor.setPower(-0.3);
        sleep(750);

        AutoFunc.MoveForwards(3000,0.9);

        boxFlipServo.setPower(0.7);
        sleep(1000);

    }


}
*/