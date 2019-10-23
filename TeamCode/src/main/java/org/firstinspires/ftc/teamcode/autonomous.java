package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubAssembly.Claimer.GrabberControl;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.Utilities.UserControl;

@Autonomous(name = "Autonomous", group = "Auto")
public class autonomous extends LinearOpMode {

    DriveControl Drive = new DriveControl();
    GrabberControl Grabber = new GrabberControl();

    //State setup
    private void newState(State newState) {
        mCurrentState = newState;
        Drive.stop();
        Drive.TimeDelay(0.1);
        //resetClock();
    }

    private enum State {
        Initial,
        DrivetoQuarry,
        GrabSkystone,
        MovetoLine,
        Stop
    }

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

        bAnswer = User.getYesNo("Wait?");
        AllianceColor = User.getRedBlue("Alliance Color");


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
                    newState(State.DrivetoQuarry);
                    break;
                case DrivetoQuarry:
                    telemetry.addLine("Drive to Quarry");
                    telemetry.update();
                    Drive.moveForwardDistance(0.5,71.2);
                    newState(State.GrabSkystone);
                    break;
                case GrabSkystone:
                    telemetry.addLine("Grab Skystone");
                    telemetry.update();
                    Grabber.open();
                    Drive.moveForwardDistance(0.5,5);
                    Grabber.close();
                    newState(State.MovetoLine);
                    break;
                case MovetoLine:
                    telemetry.addLine("MovetoLine");
                    telemetry.update();
                    Drive.moveBackwardDistance(0.5,10.16);
                    if (AllianceColor = true) {
                        Drive.turnRightDistance(0.5,50);
                    if (AllianceColor = false){
                        Drive.turnLeftDistance(0.5,50);
                    }
                    Drive.moveForwardDistance(0.5, 76.2);
                    //Drive until sees red or blue line
                    newState(State.Stop);
                    break;
                    }
                case Stop:
                    telemetry.addLine("Stop");
                    telemetry.update();
                    break;
            }
        }
    }
}