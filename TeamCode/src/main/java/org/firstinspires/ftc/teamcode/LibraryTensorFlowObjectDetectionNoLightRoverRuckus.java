/* Copyright (c) 2018 FIRST. All rights reserved.
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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.LinkedList;
import java.util.List;

/**
 * This Our OpMode that illustrates TensorFlow Object Detection API to determine the position of the
 * gold and silver minerals. For this Library we set the phone light to be off.
 */
public class LibraryTensorFlowObjectDetectionNoLightRoverRuckus {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // Set Vuforia Key so it knows what phone it is connecting to
    private static final String VUFORIA_KEY = "Aa4mtdP/////AAABmSRcR7UP9kS4nIeX1am8Tf5TlWuaSoXF9p9tlyFSx0zDxT39pe+kg1dseqSvlAQBMws92KngQN7wl3RHkCgjre8b+A9RXXtGx0mlQ1PWbMIf4AlDdHncv6ERajxzi+HwOgFkMt44eQ9gVLBLUvxzDepzfZaMSfalcWz3qtbhq8hH2R3npGb+p2x6XVY6IWZSwkKpnCFVddAhsyuToQ/S5ndIkeB2O4mquvWESjFDc6ALl/SU7Rcg5Qb/chtv2dK+EWkcaf+XSjzn7KvOsaykUeOk2ChCIEQizneBH0ILH28lPMGjxTky7qnTf+5Jb/IHpd64ZtTZN9Q2Nyrlce1750yUVtnqSxRdUPPaJTiBQrKo";
    HardwareBeep robot;
    Telemetry telemetry;
    ElapsedTime timer;
    // Set Vuforia as a Localizer
    private VuforiaLocalizer vuforia;
    // Set tfod to the TensorFlowObjectDetector
    private TFObjectDetector tfod;

    /**
     * This method sets the hardware map and telemetry for TensorFlow
     *
     * @param newHardwareBeep This is the hardware we use to initialize TensorFlow to the phone
     * @param newTelemetry    This is the parameter that calls the telemetry for us to use
     */
    public LibraryTensorFlowObjectDetectionNoLightRoverRuckus(HardwareBeep newHardwareBeep, Telemetry
            newTelemetry) {

        robot = newHardwareBeep;
        telemetry = newTelemetry;

    }

    /**
     * This method initializes vuforia and returns the gold mineral position
     *
     * @return Returns gold mineral position
     */
    public String findMineral() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        // Turn on the light on phone to make mineral visible
        phoneLight(false);

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
        String goldPosition = "";

        // sets start time to read in milliseconds
        startTime = System.currentTimeMillis();

        // sets the TensorFlow to read the mineral for at least 3 seconds to verify that it is the
        // correct mineral
        while (System.currentTimeMillis() < (startTime + 2000)) { /**DEBUG CHANGED TO 30000*/

            // sets gold position values to the read mineral function
            goldPosition = readMineral();

            // this if statement waits until the current gold position is the same as the previous
            // position
            if (goldPosition == previousPosition) {
                // if the mineral position is not the same
            } else {
                previousPosition = goldPosition;
                // restarts the start time
                startTime = System.currentTimeMillis();

            }
            // telemetry
            telemetry.addData("StartTime: ", startTime);
            telemetry.addData("CurrentTime: ", System.currentTimeMillis());
            telemetry.addData("Prev Position:  ", previousPosition);
            telemetry.addData("Gold Position:  ", goldPosition);
            telemetry.update();
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        telemetry.addData("Mineral Position: ", goldPosition);
        telemetry.update();
        // sets it to keep the phone light off
        phoneLight(false);
        return goldPosition;
    }

    /**
     * This method has an algorithm that reads the right two minerals.
     *
     * @return Returns the current gold position.
     */
    public String readMineral() {
        String currentPos = "";
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // while mineral position is not found and the timer counts 6 seconds
        while (currentPos == "" && timer.seconds() < 2) { /**DEBUG CHANGED TO 600 */
            // getUpdatedRecognitions() will return null if no new information is available since
            if (tfod != null) {
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                // if when it updates it sees an object
                if (updatedRecognitions != null) {
                    // update telemetry to read what mineral is found
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // if the object identification is greater or equal
                    if (updatedRecognitions.size() >= 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        // created a linked list which sorts the values that Tensor Flow reads.
                        // It gets the values by reading the bottom left corner of the minerals.
                        LinkedList<Recognition> recognitionLinkedList = new LinkedList<Recognition>();

                        for (Recognition recognition : updatedRecognitions) {

                            // if there are no minerals in the Linked List or if the bottom value
                            // is greater than the first than it moves that mineral value to the
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
                            telemetry.addData("mineral type", recognitionLinkedList.get(1).getLabel());
                            telemetry.addData("mineral bottom", recognitionLinkedList.get(i).getBottom());
                            telemetry.addData("List size", recognitionLinkedList.size());
                            telemetry.update();

                            // finding mineral value
                            recognition = recognitionLinkedList.get(i);
                            // if Tensor Flow reads the mineral as a gold mineral than it obtains
                            // the left x value
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                                // else if it reads 1 silver mineral it reads the left x value
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                                // else if it reads 2 silver mineral than it gets the two left x
                                // values of both minerals
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        // this is where it calculates the correct gold mineral position

                        // if it reads two silver minerals than it sets the current gold position as
                        // LEFT
                        if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            currentPos = "LEFT";
                        }
                        // if it reads a gold and silver mineral than it goes into this function
                        if (goldMineralX != -1 && silverMineral1X != -1) {
                            // if it reads the gold mineral as greater than the silver mineral than
                            // it sets the current gold mineral position as RIGHT
                            if (goldMineralX > silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                currentPos = "RIGHT";
                                //if the gold mineral is not greater than the silver mineral than
                                // it sets the current gold mineral position as CENTER
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                currentPos = "CENTER";
                            }
                        }
                    }
                }
            }
        }
        // returns the current gold mineral position
        return currentPos;
    }

    /**
     * This method initializes the Tensor Flow program on the phone
     */
    private void initVuforia() {
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * This method turns on the the phone light
     *
     * @param on This is the value we use to set the phone light to "ON"
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
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}