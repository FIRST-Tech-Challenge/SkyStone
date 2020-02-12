package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//This part is importing information from other programs
import org.firstinspires.ftc.teamcode.SubAssembly.FoundationGrabber.FoundationGrabberControl;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.Utilities.UserControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Grabber.GrabberControl;


@Autonomous(name = "Foundation Autonomous", group = "Auto")
public class foundationAuto extends LinearOpMode{
    //This gives the control programs shortened names to refer to them in this program
    DriveControl Drive = new DriveControl();
    FoundationGrabberControl FoundationGrabber = new FoundationGrabberControl();
    GrabberControl Grabber = new GrabberControl();

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
        MoveToFoundation,
        GrabFoundation,
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
        FoundationGrabber.init(this);
        Grabber.init(this);

        // get user input
        boolean bAnswer;
        boolean AllianceColor;
        boolean bridgeanswer;
        boolean stoneanswer;

        //This asks whether you want to delay start or not and whether you are red or blue
        bAnswer = User.getYesNo("Wait?");
        AllianceColor = User.getRedBlue("Alliance Color");
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
                    newState(State.MoveToFoundation);
                    break;


                case MoveToFoundation:
                    telemetry.addLine("Move to Foundation");
                    telemetry.update();
                    Drive.moveBackwardDistance(0.7,72);
                    newState(State.GrabFoundation);
                    break;


                case GrabFoundation:
                    FoundationGrabber.close();
                    Drive.TimeDelay(1.0);
                    telemetry.addLine("Reposition Foundation");
                    telemetry.update();
                    Drive.moveForwardDistance(0.8,77);
                    if (AllianceColor == false) {
                        Drive.turnLeftDistance(0.8, 45 * 5. / 9.);
                    }
                    else{
                        Drive.turnRightDistance(0.8, 45 * 5. / 9.);
                    }
                    FoundationGrabber.open();
                    if (AllianceColor == false) {
                        Drive.turnLeftDistance(0.8, 80 * 5. / 9.);
                    }
                    else {
                        Drive.turnRightDistance(0.8, 80 * 5. / 9.);
                    }
                    newState(State.Park);
                    break;


                case Park:
                    telemetry.addLine("Park");
                    if (AllianceColor == true) {
                        if (bridgeanswer == true) {
                            Drive.moveForwardDistance(0.8, 45);
                            Drive.strafeRightDistance(0.8, 90);
                            Drive.DriveUntilColor(0.5);
                        } else {
                            Drive.DriveUntilColor(0.5);
                        }
                    }
                    else {
                        if (bridgeanswer == true){
                            Drive.moveForwardDistance(0.8,45);
                            Drive.turnLeftDistance(0.8,50);
                            Drive.moveForwardDistance(0.8, 55);
                            Drive.turnRightDistance(0.8,50);
                            Drive.DriveUntilColor(0.5);
                        }
                        else {
                            Drive.DriveUntilColor(0.5);
                        }
                    }
                    newState(State.Stop);
                    break;


                case Stop:
                    telemetry.addLine("Stop");
                    telemetry.update();
                    break;
            }
        }
    }


}
