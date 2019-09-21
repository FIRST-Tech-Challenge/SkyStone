/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.LinkedList;
import java.util.List;

/**
 * This Our OpMode that illustrates TensorFlow Object Detection API to determine the position of the
 * stones. This library sets the phone light to be on.
 */
public class LibraryTensorFlowObjectDetectionNoLight {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    // Set Vuforia Key so it knows what phone it is connecting to
    private static final String VUFORIA_KEY = "Aa4mtdP/////AAABmSRcR7UP9kS4nIeX1am8Tf5TlWuaSoXF9p9tlyFSx0zDxT39pe+kg1dseqSvlAQBMws92KngQN7wl3RHkCgjre8b+A9RXXtGx0mlQ1PWbMIf4AlDdHncv6ERajxzi+HwOgFkMt44eQ9gVLBLUvxzDepzfZaMSfalcWz3qtbhq8hH2R3npGb+p2x6XVY6IWZSwkKpnCFVddAhsyuToQ/S5ndIkeB2O4mquvWESjFDc6ALl/SU7Rcg5Qb/chtv2dK+EWkcaf+XSjzn7KvOsaykUeOk2ChCIEQizneBH0ILH28lPMGjxTky7qnTf+5Jb/IHpd64ZtTZN9Q2Nyrlce1750yUVtnqSxRdUPPaJTiBQrKo";
    HardwareBeepTest robot;
    Telemetry telemetry;
    ElapsedTime timer;
    // Set Vuforia as a Localizer
    private VuforiaLocalizer vuforia;
    // Set tfod to the TensorFlowObjectDetector
    private TFObjectDetector tfod;

    /**
     * Sets the robot and telemetry for the program.
     *
     * @param newHardwareBeepTest A new variable for Hardware Beep
     * @param newTelemetry    A call to use telemetry
     */
    public LibraryTensorFlowObjectDetectionNoLight(HardwareBeepTest newHardwareBeepTest, Telemetry
            newTelemetry) {

        robot = newHardwareBeepTest;
        telemetry = newTelemetry;

    }

    /**
     * This method starts up the phone light and reads the skystone position.
     *
     * @return This return function sends back the skystone position
     */
    public String findSkystone() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        // Turn on the light on phone to make the stones visible
        phoneLight(true);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Activate Tensor Flow Object Detection
        if (tfod != null) {
            tfod.activate();
        }
        long startTime = 0;
        String previousPosition = "";
        String SkystonePosition = "";

        // sets start time to read in milliseconds
        startTime = System.currentTimeMillis();

        // sets the TensorFlow to read the mineral for at least 3 seconds to verify that it is the
        // correct mineral
        while (System.currentTimeMillis() < (startTime + 7000)) { /**DEBUG CHANGED TO 30000*/

            // sets skystone position values to the read skystone function
            SkystonePosition = readSkystone();

            // this if statement waits until the current skystone position is the same as the previous
            // position
            if (SkystonePosition == previousPosition) {
                // if the skystone position is not the same
            } else {
                previousPosition = SkystonePosition;
                // restarts the start time
                startTime = System.currentTimeMillis();

            }
            // telemetry
            telemetry.addData("StartTime: ", startTime);
            telemetry.addData("CurrentTime: ", System.currentTimeMillis());
            telemetry.addData("Prev Position:  ", previousPosition);
            telemetry.addData("Skystone Position:  ", SkystonePosition);
            telemetry.update();
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        telemetry.addData("Skystone Position: ", SkystonePosition);
        telemetry.update();
        // sets it to keep the phone light off
        phoneLight(false);
        return SkystonePosition;
    }

    // read skystone function
    public String readSkystone() {
        String currentPos = "";
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // while mineral position is not found and the timer counts 6 seconds
        while (currentPos == "" && timer.seconds() < 6) { /**DEBUG CHANGED TO 600 */
            // getUpdatedRecognitions() will return null if no new information is available since

            if (tfod != null) {
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                // if when it updates it sees an object
                if (updatedRecognitions != null) {
                    // update telemetry to read the stone
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // if the object identification is greater or equal
                    if (updatedRecognitions.size() >= 2) {
                        int Skystone1X = -1;
                        int stone1X = -1;
                        int stone2X = -1;
                        // created a linked list which sorts the values that Tensor Flow reads.
                        // It gets the values by reading the bottom left corner of the skystones.
                        LinkedList<Recognition> recognitionLinkedList = new LinkedList<Recognition>();

                        for (Recognition recognition : updatedRecognitions) {

                            // if there are no stones in the Linked List or if the bottom value
                            // is greater than the first than it moves that stone value to the
                            // from of the Linked List
                            if ((recognitionLinkedList.isEmpty()) ||
                                    (recognition.getBottom() > recognitionLinkedList.getFirst().getBottom())) {

                                recognitionLinkedList.addFirst(recognition);
                            } else {
                                //Add the recognition after the one that its directly lower than
                                for (int i = 0; i < recognitionLinkedList.size(); i++) {
                                    if (recognition.getBottom() > recognitionLinkedList.get(i).getBottom()) {
                                        recognitionLinkedList.add(i, recognition);
                                        break;
                                    }
                                    //if we're analyzing the last element, then add to the end of list
                                    if (i == recognitionLinkedList.size() - 1) {
                                        recognitionLinkedList.add(recognition);
                                        break;
                                    }
                                }
                            }
                        }

                        Recognition recognition = null;
                        for (int i = 0; i < 2; i++) {
                            telemetry.addData("iterate over first two elements: element ", i);
                            telemetry.addData("stone type", recognitionLinkedList.get(1).getLabel());
                            telemetry.addData("stone bottom", recognitionLinkedList.get(i).getBottom());
                            telemetry.addData("List size", recognitionLinkedList.size());
                            telemetry.update();

                            // finding skystone value
                            recognition = recognitionLinkedList.get(i);
                            // if Tensor Flow reads the stone as the skystone than it obtains
                            // the left x value
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                Skystone1X = (int) recognition.getLeft();
                                // else if it reads 1 stone it reads the left x value
                            } else if (stone1X == -1) {
                                stone1X = (int) recognition.getLeft();
                                // else if it reads 2 stones than it gets the two left x
                                // values of both stones
                            } else {
                                stone2X = (int) recognition.getLeft();
                            }
                        }
                        // this is where it calculates the correct skystone position

                        // if it reads two stones than it sets the current skystone position as
                        // LEFT
                        if (Skystone1X == -1 && stone1X != -1 && stone2X != -1) {
                            telemetry.addData("Skystone Position", "Pos 3");
                            currentPos = "Pos 3";
                        }
                        // if it reads a skystone and a stone than it goes into this function
                        if (Skystone1X != -1 && stone1X != -1) {
                            // if it reads the skystone as greater than the stone than
                            // it sets the current skystone position as RIGHT
                            if (Skystone1X > stone1X) {
                                telemetry.addData("Skystone Position", "Pos 1");
                                currentPos = "Pos 1";
                                //if the skystone is not greater than the stone than
                                // it sets the current skystone position as CENTER
                            } else {
                                telemetry.addData("Skystone Position", "Pos 2");
                                currentPos = "Pos 2";
                            }
                        }
                    }
                }
            }
        }
        // returns the current skystone position
        return currentPos;
    }

    /**
     * That method starts up vuforia on the phone
     */
    private void initVuforia() {
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    // this function is for the phone light

    /**
     * This method is called to start up the phone light
     *
     * @param on This parameter is used to start up the phone light
     */
    private void phoneLight(boolean on) {

        // if you set the phone light to true than it turns on the phone flashlight
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(on);

    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
