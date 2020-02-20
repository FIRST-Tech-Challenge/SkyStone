package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import com.qualcomm.robotcore.util.ElapsedTime;

//This part is importing information from other programs
import org.firstinspires.ftc.teamcode.SubAssembly.Lift.LiftControl;
import org.firstinspires.ftc.teamcode.SubAssembly.FoundationGrabber.FoundationGrabberControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Grabber.GrabberControl;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.Utilities.UserControl;
import org.firstinspires.ftc.teamcode.Utilities.ConceptVuforiaSkyStoneNavigationWebcam;


@Autonomous(name = "Skystone Autonomous", group = "Auto")
public class SkystoneAutonomous extends LinearOpMode{
    //This gives the control programs shortened names to refer to them in this program
    DriveControl Drive = new DriveControl();
    GrabberControl Grabber = new GrabberControl();
    ConceptVuforiaSkyStoneNavigationWebcam Webcam = new ConceptVuforiaSkyStoneNavigationWebcam();
    FoundationGrabberControl FoundationGrabber = new FoundationGrabberControl();
    LiftControl Lift = new LiftControl();

    //State setup
    private void newState(State newState) {
        mCurrentState = newState;
        Drive.stop();
        Drive.TimeDelay(0.1);
        //resetClock();
    }

    //This is a list of all of the states
    private enum State {
        Initial,
        GrabStone,
        MoveToBuildZone,
        TurnFoundation,
        AutoPark,
        Park,
        Stop
    }

    //This is a list of all the possible skystone positions
    private enum SkystonePosition {
        R1,
        R2,
        R3,
        B1,
        B2,
        B3,
    }

    private enum BlueSkystonePosition {
        B1,
        B2,
        B3,
    }
    //This sets the skystone to a default position
    private SkystonePosition Skystone = SkystonePosition.R1;
    //private BlueSkystonePosition BlueSkystone = BlueSkystonePosition.B1;

    //This sets the default starting state
    private State mCurrentState = State.Initial;

    @Override
    public void runOpMode() throws InterruptedException {
        // declare local variables
        double speed = 0.3;

        // display welcome message
        telemetry.setAutoClear(false);
        telemetry.addLine("Autonomous");
        telemetry.update();

        // create and initialize sub-assemblies
        UserControl User = new UserControl();
        User.init(this);
        Drive.init(this);
        Grabber.init(this);
        FoundationGrabber.init(this);
        Lift.initialize(this);

        // get user input
        boolean bAnswer;
        boolean AllianceColor;
        boolean willPark;

        //This asks whether you want to delay start or not and whether you are red or blue
        bAnswer = User.getYesNo("Wait?");
        AllianceColor = User.getRedBlue("Alliance Color");
        willPark = User.getPark("Park?");

        /*This will use the skystone position determined by vuforia and the
        alliance color to determine the skystone position*/
        if (Webcam.PRS == Webcam.PRS.CENTER && AllianceColor == true){
            Skystone = SkystonePosition.R2;
        } else if (Webcam.PRS == Webcam.PRS.LEFT && AllianceColor == true){
            Skystone = SkystonePosition.R3;
        } else if (Webcam.PRS == Webcam.PRS.RIGHT && AllianceColor == true){
            Skystone = SkystonePosition.R1;
        } else if (Webcam.PBS == Webcam.PBS.CENTER && AllianceColor == false){
            Skystone = SkystonePosition.B2;
        } else if (Webcam.PBS == Webcam.PBS.RIGHT && AllianceColor == false){
            Skystone = SkystonePosition.B3;
        } else {
            Skystone = SkystonePosition.B1;
        }

        // wait for PLAY button to be pressed on driver station
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        // begin autonomous actions
        telemetry.setAutoClear(false);
        newState(State.Initial);

        while (opModeIsActive() && mCurrentState != State.Stop) {

            //now = runtime.seconds() - lastReset;

            //state switch
            switch (mCurrentState) {
                /*Initializes auto and waits for the time delay*/
                case Initial:
                    telemetry.addLine("Initial");
                    telemetry.update();
                    if (bAnswer) {
                        telemetry.addLine("wait for 5 seconds");
                        telemetry.update();
                        Drive.TimeDelay(5.0);
                    }
                    if (willPark == true){
                        newState(State.Park);
                    }
                    else {
                        newState(State.GrabStone);
                    }
                    break;


                case GrabStone:
                    telemetry.addLine("grab skystone");
                    telemetry.update();
                    Grabber.open();
                    Grabber.Pos1();
                    Drive.moveForwardDistance(0.8, 60);
                    if (Skystone == SkystonePosition.R3 || Skystone == SkystonePosition.B1) {
                        Drive.moveForwardDistance(0.8, 20);
                    } else if (AllianceColor == true){
                        Drive.strafeLeftDistance(0.8, 20.32);
                        Drive.moveForwardDistance(0.8, 20);
                    } else {
                        Drive.strafeRightDistance(0.8, 20.32);
                        Drive.moveForwardDistance(0.8, 20);
                    }
                    Grabber.close();
                    Drive.TimeDelay(0.5);
                    newState(State.MoveToBuildZone);
                    break;


                case MoveToBuildZone:
                    Drive.moveBackwardDistance(0.8,40);
                    if (AllianceColor == true) {
                        Drive.turnRightAngle(0.5, 90);
                    }
                    else {
                        Drive.turnLeftAngle(0.5,90);
                    }
                    Drive.moveForwardDistance(0.8, 175);
                    if (Skystone != SkystonePosition.R3 && Skystone != SkystonePosition.B1)
                        Drive.moveForwardDistance(0.8, 20.32);
                    newState(State.TurnFoundation);
                    break;


                case TurnFoundation:
                    if (AllianceColor == false) {
                        Drive.turnRightAngle(0.5, 90);
                    }
                    else {
                        Drive.turnLeftAngle(0.5,90);
                    }
                    Lift.MoveUpTime(0.4);
                    Drive.moveForwardDistance(0.5, 35);
                    Grabber.open();
                    Drive.TimeDelay(1.0);
                    Drive.moveBackwardDistance(0.8, 25);
                    if (AllianceColor == false) {
                        Drive.turnRightAngle(0.5, 90);
                    }
                    else {
                        Drive.turnLeftAngle(0.5,90);
                    }
                    Lift.MoveDownTime(0.4);
                    //FoundationGrabber.open();
                    newState(State.AutoPark);
                    break;


                case AutoPark:
                    telemetry.addLine("Park");
                    telemetry.update();
                    /*Drive.moveForwardDistance(0.8, 30.0);
                    Drive.turnLeftDistance(0.8, 100);
                    Lift.MoveUpTime(0.4);
                    Drive.moveForwardDistance(0.8,60.0);
                    Grabber.open();
                    Drive.moveBackwardDistance(0.8,60);
                    Lift.MoveDownTime(0.4);
                    /*Drive backwards until under skybridge*/
                    Drive.moveForwardDistance(0.8,100);
                    newState(State.Stop);
                    break;

                case Park:
                    telemetry.addLine("Park");
                    telemetry.update();
                    Drive.moveForwardDistance(0.8, 60);
                    newState(State.Stop);
                    break;


                case Stop:
                    telemetry.addLine("Stop");
                    telemetry.update();
                    break;
            }
        }

        // ensure proper closure of subassemblies
        Lift.finalize();
    }
}
