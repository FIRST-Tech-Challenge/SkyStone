package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//This part is importing information from other programs
import org.firstinspires.ftc.teamcode.SubAssembly.Lift.LiftControl;
import org.firstinspires.ftc.teamcode.SubAssembly.FoundationGrabber.FoundationGrabberControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Grabber.GrabberControl;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.Utilities.UserControl;

@Autonomous(name = "Basic Autonomous", group = "Auto")
public class basicAutonomous extends LinearOpMode{
    //This gives the control programs shortened names to refer to them in this program
    DriveControl Drive = new DriveControl();
    GrabberControl Grabber = new GrabberControl();
    FoundationGrabberControl FoundationGrabber = new FoundationGrabberControl();
    LiftControl Lift = new LiftControl();

    //State setup
    private void newState(State newState) {
        mCurrentState = newState;
        Drive.stop();
        Drive.TimeDelay(0.1);
    }

    //This is a list of all of the states
    private enum State {
        Initial,
        MoveToStone,
        GrabStone,
        MoveToBuildZone,
        MoveToFoundation,
        PlaceStone,
        ParkFromQuarry,
        Park,
        Stop
    }

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
        boolean bridgeanswer;

        //This asks whether you want to delay start or not and whether you are red or blue
        bAnswer = User.getYesNo("Wait?");
        AllianceColor = User.getRedBlue("Alliance Color");
        willPark = User.getPark("Park?");
        bridgeanswer = User.getPos("Bridge or Wall?");

        // wait for PLAY button to be pressed on driver station
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        // begin autonomous actions
        telemetry.setAutoClear(false);
        newState(State.Initial);

        while (opModeIsActive() && mCurrentState != State.Stop) {

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
                    //checks to see if running full auto or just parking
                    if (willPark == true){
                        newState(State.ParkFromQuarry);
                    }
                    else {
                        newState(State.MoveToStone);
                    }
                    break;

                case MoveToStone:
                    telemetry.addLine("move to stone");
                    telemetry.update();
                    Grabber.open();
                    Grabber.Pos1();
                    Drive.moveForwardDistance(0.8, 80);
                    newState(State.GrabStone);
                    break;

                case GrabStone:
                    telemetry.addLine("grab stone");
                    telemetry.update();
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
                    newState(State.MoveToFoundation);
                    break;


                case MoveToFoundation:
                    if (AllianceColor == true) {
                        Drive.turnLeftAngle(0.5, 90);
                    }
                    else {
                        Drive.turnRightAngle(0.5,90);
                    }
                    Lift.MoveUpTime(0.4);
                    Drive.moveForwardDistance(0.5, 35);
                    newState(State.PlaceStone);

                case PlaceStone:
                    Grabber.open();
                    Drive.TimeDelay(1.0);
                    newState(State.Park);
                    break;


                case Park:
                    Drive.moveBackwardDistance(0.8, 25);
                    Lift.MoveDownTime(0.4);
                    Grabber.close();
                    //checks to see where to park
                    if (bridgeanswer == true) {
                            if (AllianceColor == true) {
                                Drive.turnLeftAngle(0.5, 90);
                                Drive.moveForwardDistance(0.8,50);
                                //Drive.strafeRightDistance(0.8,15);
                            }
                            else {
                                Drive.turnRightAngle(0.5,90);
                               //Drive.strafeLeftDistance(0.8,15);
                                Drive.moveForwardDistance(0.8,50);
                            }
                            Drive.moveForwardDistance(0.8, 50);
                        }
                    else {
                            Drive.moveBackwardDistance(0.8, 60);
                            if (AllianceColor == true) {
                                Drive.turnLeftAngle(0.5, 90);
                            }
                            else {
                                Drive.turnRightAngle(0.5,90);
                            }
                            Drive.moveForwardDistance(0.8, 100);
                        }
                    newState(State.Stop);
                    break;


                case ParkFromQuarry:
                    telemetry.addLine("Park");
                    telemetry.update();
                    if (bridgeanswer == true) {
                        Drive.moveForwardDistance(0.8, 65);
                        if (AllianceColor == true) {
                            Drive.turnLeftAngle(0.8, 90);
                        } else {
                            Drive.turnRightAngle(0.8, 90);
                        }
                        Drive.moveForwardDistance(0.8, 70);
                    }
                    else
                        if (AllianceColor == true){
                            Drive.strafeLeftDistance(0.8,90);
                        }
                        else {
                            Drive.strafeRightDistance(0.8,90);
                        }
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