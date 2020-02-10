package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.Utilities.ConceptVuforiaSkyStoneNavigationWebcam;
import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@Autonomous(name = "Vuforia Test", group = "Test")
public class VuforiaTest extends LinearOpMode {

    DriveControl Drive = new DriveControl();
    private ConceptVuforiaSkyStoneNavigationWebcam Webcam = new ConceptVuforiaSkyStoneNavigationWebcam();
    UserControl User = new UserControl();


    /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);*/

    private void newState(State newState) {
        mCurrentState = newState;
        Drive.stop();
        Drive.TimeDelay(0.1);
        //resetClock();
    }

    //This is a list of all of the states
    private enum State {
        Initial,
        VuforiaTest,
        LetMeSeeTelemetry,
        Stop
    }

    private enum SkystonePosition {
        R1,
        R2,
        R3,
        B1,
        B2,
        B3,
    }

    private SkystonePosition Skystone = SkystonePosition.B1;

    private State mCurrentState = State.Initial;

    @Override
    public void runOpMode() throws InterruptedException {
        // create and initialize sub-assemblies
        Drive.init(this);
        Webcam.init();
        User.init(this);

        boolean AllianceColor;

        /*AllianceColor = User.getRedBlue("Alliance Color");
        if (AllianceColor == true) {
            if (Webcam.PRS == Webcam.PRS.RIGHT) {
                Skystone = SkystonePosition.R1;
            } else if (Webcam.PRS == Webcam.PRS.CENTER) {
                Skystone = SkystonePosition.R2;
            } else {
                Skystone = SkystonePosition.R3;
            }
        } else if (AllianceColor == false) {
            if (Webcam.PBS == Webcam.PBS.LEFT) {
                Skystone = SkystonePosition.B1;
            } else if (Webcam.PBS == Webcam.PBS.CENTER) {
                Skystone = SkystonePosition.B2;
            } else {
                Skystone = SkystonePosition.B3;
            }
        }*/

        // wait for PLAY button to be pressed on driver station
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        newState(State.Initial);

        while (opModeIsActive() && mCurrentState != State.Stop) {
            switch (mCurrentState) {
                case Initial:
                    telemetry.addLine("Initial");
                    telemetry.update();
                    newState(State.VuforiaTest);
                    break;
                case VuforiaTest:
                    telemetry.addData("Red Skystone Position", Webcam.PRS);
                    if (Webcam.PBS == Webcam.PBS.LEFT)
                        telemetry.addLine("Blue Skystone Left");
                    else if (Webcam.PBS == Webcam.PBS.CENTER)
                        telemetry.addLine("Blue Skystone Center");
                    else
                        telemetry.addLine("Blue Skystone Right");

                    if (Webcam.PRS == Webcam.PRS.RIGHT)
                        telemetry.addLine("Red Skystone Right");
                    else if (Webcam.PRS == Webcam.PRS.CENTER)
                        telemetry.addLine("Red Skystone Center");
                    else
                        telemetry.addLine("Red Skystone Left");
                        // express the rotation of the robot in degrees.
                    newState(State.LetMeSeeTelemetry);
                case LetMeSeeTelemetry:
                    Drive.TimeDelay(10);
                case Stop:
                    telemetry.addLine("Stop");
                    telemetry.update();
                    break;
            }
        }
    }
}
