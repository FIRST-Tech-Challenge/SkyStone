package org.firstinspires.ftc.teamcode.autoOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.auto.ChassisStandard;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;

import java.util.List;

/**
 * This just runs from the position closest to the crater, into the crater.
 */
public abstract class TurnTest extends ChassisStandard {

    public TurnTest(ChassisConfig config) {
        super(config);
    }


    private final int SCREEN_WIDTH = 600;
    private int stoneconfig;
    private float lastStone = -1;

    /**a
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop () {

        if (madeTheRun == false) {
            /*

            turnRight(90);
            sleep(1000);
            turnLeft(90);*/

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        if (recognition.getLabel() == "Skystone") {
                            
                            lastStone = recognition.getLeft();
                            if (recognition.getLeft() < (SCREEN_WIDTH / 3)) {
                                stoneconfig = 1;
                            } else if (recognition.getLeft() > (SCREEN_WIDTH * 2.0 / 3.0)) {
                                stoneconfig = 3;
                            } else {
                                stoneconfig = 2;
                            }
                        }

                    }
                    telemetry.update();
                }
            }

            //madeTheRun = true;
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "time: " + runtime.toString());
        telemetry.addData("Gyro", "angle: " + this.getGyroscopeAngle());
        telemetry.addData("Status", "madeTheRun=%b", madeTheRun);
        telemetry.addData("StatusYo", "madeTheRun=freg");
        telemetry.addData("stoneconfig", "config=%d, left=%f" , stoneconfig, lastStone);
        //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
        //        recognition.getLeft(), recognition.getTop());
    }
}

