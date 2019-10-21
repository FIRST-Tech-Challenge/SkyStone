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

package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.RuckusUtils;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous
public class ArmstrongMarkerLower2nd extends AutoOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public static final int LEFT = 1;
    public static final int CENTER = 2;
    public static final int RIGHT = 3;
    public static int mineralOri;
    Armstrong armstrong;


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "ATU9MNz/////AAABmdp9yZ8JdEGjpiGfxU8g64YjAQPwRcIIIqytyWu9HmjEkTELwI1JsCtkFv/I4k2S8KXjgWFB61R+GwLPvY3T1EyQmpV/UFfaSEqcJLpT++NbMjv5JkXg3JG92Ga+RnHYS3WaTBgRZexhqar4QNK4exrzUQUJjy2ntF2Afb+ENqH4glLQW85aM0BA4+8WMjcplpZ5WbhJ82ruz0ikcpy8bffFnhd+pN1/xficoB/Szcx5lt1SmKzVjbYkktmVd8qS6qGd8yVH1DydPQlP6njcUDllIc1a3oAO5zmWTFoxfaknDOm2bXka6V2Qht6pD7pl1tSP3vgeCZPM0fKSowfy0MoFVzsuuBvwqloB4Obt4NDT";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    @Override
    public void runOp() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        armstrong = new Armstrong();
//        armstrong.motorL.setReverse(!armstrong.motorL.isReversed());

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        clearTimer(1);// start tensor flow timer
        while (getSeconds(1) < 4){
            RC.t.addData(getSeconds(1));
        }



        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();

        }

        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    RC.t.addData(getSeconds(1));
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                            mineralOri = RuckusUtils.getCubePostition(goldMineralX, silverMineral1X, silverMineral2X);
                        }
                    }
                    if (updatedRecognitions.size() != 3 || mineralOri == -1 && getSeconds(1) > 4) {
                        mineralOri = Robot.CENTRE;
                        RC.t.addData("Automatically Centre");
                        break;
                    }
                    if (mineralOri > -1) {
                        RC.t.addData("Mineral Config", mineralOri);
                        break;
                    }
                }//size
                //size null

            }

        }


        if (tfod != null) {
            tfod.shutdown();
        }

        clearTimer(2);// rack and pinon timer


        while (armstrong.magnetSensor.getState() && getSeconds(2)<7) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
            armstrong.lifterUp();
            armstrong.collectServoLeftSlow();
            armstrong.collectServoRightSlow();
            RC.t.addData(getSeconds(2));
            //new untested
        }


        telemetry.addData("Digital Touch", "Is Pressed");
        armstrong.lifterStop();
        armstrong.unlatch();
        sleep(2000);
        telemetry.addData("un", "latched");

        if (mineralOri == Robot.LEFT){

            armstrong.forwardDistance(300, 0.5);
            RC.t.addData("Found that its lefttt");
            armstrong.LeftSample();
            sleep(1000);
            armstrong.forwardDistance(300, 0.5);
            armstrong.LeftWingStore();
            sleep(600);
            //sample
            armstrong.forwardDistance(250, 0.5);
            //marker
            armstrong.markWallDown();
            sleep(1000);
            telemetry.addData("Status", "WallDown");
        }
        else if (mineralOri == Robot.CENTRE){
            armstrong.forwardDistance(300, 0.5);
            RC.t.addData("Found that its centerrr");
            armstrong.MiddleSample();
            sleep(1000);
            armstrong.forwardDistance(300, 0.5);
            armstrong.LeftWingStore();
            sleep(600);
            //sample
            armstrong.forwardDistance(250, 0.5);
            //marker
            armstrong.markWallDown();
            sleep(1000);
            telemetry.addData("Status", "WallDown");
        }
        else if (mineralOri == Robot.RIGHT){
            armstrong.forwardDistance(300, 0.5);
            RC.t.addData("Found that its rightttt");
            armstrong.RightSample();
            sleep(1000);
            armstrong.forwardDistance(300, 0.5);
            armstrong.LeftWingStore();
            sleep(600);
            //sample
            armstrong.forwardDistance(250, 0.5);
            //marker
            armstrong.markWallDown();
            sleep(1000);
            telemetry.addData("Status", "WallDown");
        }


        //crater
        //add bang-bang here

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}


//                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                    //return GOLD;
//                                    telemetry.addData("Gold Mineral Position", "Left");
//                                    mineralOri = LEFT;
//                                    sleep(100);
//                                    armstrong.forward(0.5);
//                                    sleep(1000);
//                                    armstrong.stop();
//
//                                    armstrong.LeftSample();
//                                    sleep(1000);
//                                    armstrong.forward(0.5);
//                                    sleep(1000);
//                                    armstrong.stop();
//                                    armstrong.LeftWingStore();
//                                    sleep(600);
//                                    //sample
//                                    armstrong.markWallDown();
//                                    sleep(1000);
//                                    telemetry.addData("Status", "WallDown");
//                                    armstrong.backward(0.3);
//                                    sleep(500);
//                                    armstrong.stop();
//                                    return;
//                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Right");
//                                    mineralOri = RIGHT;
//                                    RC.t.addData("rightttt");
//                                    sleep(100);
//                                    armstrong.forward(0.5);
//                                    sleep(1000);
//                                    armstrong.stop();
//
//                                    armstrong.RightSample();
//                                    sleep(1000);
//                                    armstrong.forward(0.5);
//                                    sleep(1000);
//                                    armstrong.stop();
//                                    armstrong.RightWingStore();
//                                    sleep(1000);
//                                    //sample
//                                    armstrong.forward(0.5);
//                                    sleep(1000);
//                                    //marker
//                                    armstrong.markWallDown();
//                                    sleep(1000);
//                                    telemetry.addData("Status", "WallDown");
//                                    armstrong.backward(0.3);
//                                    sleep(500);
//                                    armstrong.stop();
//                                    return;
//                                } else {
//                                    telemetry.addData("Gold Mineral Position", "Center");
//                                    mineralOri = CENTER;
//                                    sleep(100);
//                                    armstrong.forward(0.5);
//                                    sleep(1000);
//                                    armstrong.stop();
//
//                                    armstrong.MiddleSample();
//                                    sleep(1000);
//                                    armstrong.forward(0.5);
//                                    sleep(1000);
//                                    armstrong.stop();
//                                    armstrong.RightWingStore();
//                                    armstrong.LeftWingStore();
//                                    sleep(600);
//                                    //sample
//                                    armstrong.markWallDown();
//                                    sleep(1000);
//                                    telemetry.addData("Status", "WallDown");
//                                    armstrong.backward(0.3);
//                                    sleep(500);
//                                    armstrong.stop();
//                                    return;
//                                }
//                            }
//                        }
//                        telemetry.update();