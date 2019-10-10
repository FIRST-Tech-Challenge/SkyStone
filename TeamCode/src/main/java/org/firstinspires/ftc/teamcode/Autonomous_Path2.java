package org.firstinspires.ftc.teamcode;
/*
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Path 2 (TR/BL)", group = "AutoPaths")
public class Autonomous_Path2 extends LinearOpMode {


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


        AutoFunc.MoveForwards(1000, 0.9);
        grabberServo.setPower(0.7);
        sleep(1000);

    }


}
*/