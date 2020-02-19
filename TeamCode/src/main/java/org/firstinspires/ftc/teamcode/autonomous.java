package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import com.qualcomm.robotcore.util.ElapsedTime;

//This part is importing information from other programs
import org.firstinspires.ftc.teamcode.SubAssembly.Grabber.GrabberControl;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.Utilities.UserControl;
import org.firstinspires.ftc.teamcode.Utilities.ConceptVuforiaSkyStoneNavigationWebcam;

import javax.crypto.spec.RC2ParameterSpec;

@Autonomous(name = "Autonomous", group = "Auto")
public class autonomous extends LinearOpMode {

    //This gives the control programs shortened names to refer to them in this program
    DriveControl Drive = new DriveControl();
    GrabberControl Grabber = new GrabberControl();
    ConceptVuforiaSkyStoneNavigationWebcam Webcam = new ConceptVuforiaSkyStoneNavigationWebcam();

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
        DrivetoQuarry,
        GrabSkystone,
        MovetoLine,
        MovetoLineStart,
        ScoreStone,
        BackupToQuarry,
        GrabSkystone2,
        MovetoLine2,
        ScoreStone2,
        Stop,
        test
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
    //This sets the skystone to a default position
    private SkystonePosition Skystone = SkystonePosition.B1;

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

        // get user input
        boolean bAnswer;
        boolean AllianceColor;

        //This asks whether you want to delay start or not and whether you are red or blue
        bAnswer = User.getYesNo("Wait?");
        AllianceColor = User.getRedBlue("Alliance Color");

        /*This will use the skystone position determined by vuforia and the
        alliance color to determine the skystone position*/
        /*if (Webcam.PS == Webcam.PS.CENTER && AllianceColor == true){
            Skystone = SkystonePosition.R2;
        } else if (Webcam.PS == Webcam.PS.LEFT && AllianceColor == true){
            Skystone = SkystonePosition.R3;
        } else if (Webcam.PS == Webcam.PS.RIGHT && AllianceColor == true){
            Skystone = SkystonePosition.R1;
        } else if (Webcam.PS == Webcam.PS.CENTER && AllianceColor == false){
            Skystone = SkystonePosition.B2;
        } else if (Webcam.PS == Webcam.PS.RIGHT && AllianceColor == false){
            Skystone = SkystonePosition.B3;
        } else {
            Skystone = SkystonePosition.B1;
        }*/

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
                    newState(State.MovetoLineStart);
                    break;
                // The robot drives forward until it reaches the quarry
                case DrivetoQuarry:
                    telemetry.addLine("Drive to Quarry");
                    telemetry.update();
                    Drive.moveForwardDistance(0.75,71.2);
                    newState(State.MovetoLine);
                    break;
                // Navigates to where the skystone is and grabs it
                case GrabSkystone:
                    telemetry.addLine("Grab Skystone");
                    telemetry.update();
                    if (Skystone == SkystonePosition.R2){
                        telemetry.addLine("R2");
                        Drive.strafeLeftDistance(0.75, 20.32);
                    } else if (Skystone == SkystonePosition.R3){
                        telemetry.addLine("R3");
                        Drive.strafeLeftDistance(0.75, 40.64);
                    } else if (Skystone == SkystonePosition.B2){
                        telemetry.addLine("B2");
                        Drive.strafeRightDistance(0.75, 20.32);
                    } else if (Skystone == SkystonePosition.B3){
                        telemetry.addLine("B3");
                        Drive.strafeRightDistance(0.75, 40.64);
                    } else if (Skystone == SkystonePosition.B1){
                        telemetry.addLine("B1");
                    } else {
                        telemetry.addLine("R1");
                    }
                    Grabber.open();
                    Drive.moveForwardDistance(0.5,5);
                    Grabber.close();
                    newState(State.MovetoLine);
                    break;
                // Turns and drives until under the skybridge
                case MovetoLine:
                    telemetry.addLine("MovetoLine");
                    telemetry.update();
                    Drive.moveBackwardDistance(0.5,10.16);
                    if (AllianceColor == true) {
                        Drive.turnRightAngle(0.5, 90);
                    }
                    else {
                        Drive.turnLeftAngle(0.5,90);
                    }
                    Drive.moveForwardDistance(0.75, 76.2);
                    /*if (Skystone == SkystonePosition.R3 || Skystone == SkystonePosition.B3){
                        Drive.moveForwardDistance(0.75, 20.32);
                    }*/
                    //Drive until sees red or blue line
                    newState(State.Stop);
                    break;

                //moves to line from starting position
                case MovetoLineStart:
                    telemetry.addLine("StrafetoLine");
                    telemetry.update();
                    Grabber.open();
                    Drive.moveForwardDistance(0.5, 85);
                    newState(State.Stop);
                    break;

                /* Drives into the building zone, leaves the skystone there, and backs up
                until under the skybridge again*/
                case ScoreStone:
                    Drive.moveForwardDistance(0.75,90);
                    Grabber.open();
                    Drive.moveBackwardDistance(0.75,90);
                    newState(State.Stop);
                    break;

                case BackupToQuarry:
                    telemetry.addLine("Backup to quarry");
                    telemetry.update();
                    Drive.moveBackwardDistance(0.5,95.2);
                    if (AllianceColor == true){
                        Drive.turnLeftAngle(0.5,90);
                    }
                    else {
                        Drive.turnRightAngle(0.75,90);
                    }
                    newState(State.GrabSkystone2);
                    break;

                case GrabSkystone2:
                    telemetry.addLine("Grab the second skystone");
                    telemetry.update();
                    Grabber.open();
                    Grabber.close();
                    newState(State.MovetoLine2);
                    break;

                case MovetoLine2:
                    telemetry.addLine("Move to line");
                    telemetry.update();
                    Drive.moveBackwardDistance(0.5,10.16);
                    if (AllianceColor == true)
                        Drive.turnRightAngle(0.5,90);
                    else{
                        Drive.turnLeftAngle(0.5,90);
                    }
                    Drive.moveForwardDistance(0.75, 95.2);
                    //Drive until sees red or blue line
                    newState(State.ScoreStone2);
                    break;

                case ScoreStone2:
                    Drive.moveForwardDistance(0.75,91.44);
                    Grabber.open();
                    Drive.moveBackwardDistance(0.75,91.44);
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