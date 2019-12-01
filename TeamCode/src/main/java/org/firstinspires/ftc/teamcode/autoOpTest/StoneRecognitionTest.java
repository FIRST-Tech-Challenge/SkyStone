package org.firstinspires.ftc.teamcode.autoOpTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;

import java.util.List;

/**
 *
 */
@Autonomous(name="Stone Recog Test", group="ZZTesting")
public class StoneRecognitionTest extends ChassisStandard {

    private final int SCREEN_WIDTH = 600;

    private String stoneconfig;

    public StoneRecognitionTest() {
        // override the default of vuforia being off.
        useVuforia = true;
        stoneconfig = "unknown";
    }


    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop () {

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.

            // TODO: use chassis function, dont call getUpdatesRecodgnitions directly.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("StoneDetect label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("StoneDetect left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("StoneDetect right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());

                    int offSet = -50;
                    int leftBorder = SCREEN_WIDTH/3 +offSet;
                    int rightBorder = (int) (SCREEN_WIDTH * 2.0 / 3.0 +offSet);

                    if (recognition.getLabel() == "Skystone") {
                        if (recognition.getLeft() < leftBorder) {
                            stoneconfig = "LEFT";
                        } else if (recognition.getLeft() > rightBorder) {
                            stoneconfig = "RIGHT";
                        } else {
                            stoneconfig = "CENTER";
                        }
                    }
                }
            }
        }

        telemetry.addData("StoneDetectLoc", "loc=%s", stoneconfig);
        printStatus();
    }
}
