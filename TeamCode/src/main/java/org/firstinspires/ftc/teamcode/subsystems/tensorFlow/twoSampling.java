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

package org.firstinspires.ftc.teamcode.subsystems.tensorFlow;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class twoSampling implements TensorFlow {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    public twoSampling(Telemetry myTelemetry, HardwareMap myHardwareMap, VuforiaLocalizer myVuforia, TFObjectDetector mytfod){
        telemetry = myTelemetry;
        hardwareMap = myHardwareMap;
        vuforia = myVuforia;
        tfod = mytfod;

        initVuforia();
        initTfod();
        tFodActivate();
    }

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "Adp/KFX/////AAAAGYMHgTasR0y/o1XMGBLR4bwahfNzuw2DQMMYq7vh4UvYHleflzPtt5rN2kFp7NCyO6Ikkqhj/20qTYc9ex+340/hvC49r4mphdmd6lI/Ip64CbMTB8Vo53jBHlGMkGr0xq/+C0SKL1hRXj5EkXtSe6q9F9T/nAIcg9Jr+OfAcifXPH9UJYG8WmbLlvpqN+QuVA5KQ6ve1USpxYhcimV9xWCBrq5hFk1hGLbeveHrKDG3wYRdwBeYv3Yo5qYTsotfB4CgJT9CX/fDR/0JUL7tE29d1v1eEF/VXCgQP4EPUoDNBtNE6jpKJhtQ8HJ2KjmJnW55f9OqNc6SsULV3bkQ52PY+lPLt1y4muyMrixCT7Lu";

    private int RUNTIME = 5000;

//    enum goldMineral  = threesampling.goldMineral;
    private goldMineral location = goldMineral.UNKNOWN;

    @Override
    public void initVuforia() {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    @Override
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    @Override
    public void tFodActivate(){
        if (tfod != null) {
            tfod.activate();
        }
    }

    @Override
    public void lookForMinerals() {
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            //Set all 3 minerals to -1. -1 indicates it is not recognized
            if (updatedRecognitions.size() == 2) {
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
                    telemetry.addData("gold", goldMineralX);
                    telemetry.addData("silver1", silverMineral1X);
                    telemetry.addData("silver2", silverMineral2X);
//                    telemetry.update();
                }
                //If a gold is seen, determine if it is on the left or in the center.
                if ((goldMineralX != -1 && silverMineral1X != -1)) {
                    telemetry.addData("hello", "world");
                    //Display order of minerals based on X coordinates
                    if (goldMineralX < silverMineral1X) {
                        telemetry.addData("Gold Mineral Position", "Left");
                        location = goldMineral.LEFT;
                    }
                    else if (goldMineralX > silverMineral1X) {
                        telemetry.addData("Gold Mineral Position", "Center");
                        location = goldMineral.CENTER;
                    }
                }
                //If it sees two silvers, the gold mineral is on the right
                else if ((silverMineral1X != -1 && silverMineral2X != -1)) {
                    telemetry.addData("Gold Mineral Position", "Right");
                    location = goldMineral.RIGHT;
                }
                //Extra condition if it doesn't see any silver minerals. Sets location to center.
                else {
                    telemetry.addData("Gold Mineral Position", "Right");
                    location = goldMineral.RIGHT;
                }
                telemetry.update();
            }
        }
    }



    @Override
    public TensorFlow.goldMineral getMineralTime() throws InterruptedException {
        RUNTIME/=10;
        double count=0;

        while (location == goldMineral.UNKNOWN && count < RUNTIME) {
            lookForMinerals();
            telemetry.addData("Gold", "%s position", location);
            telemetry.addData("Seconds", count/100);
            telemetry.update();
            Thread.sleep(10);
            count++;
        }
        telemetry.addData("Gold", "is Right");
        return location;
    }

    @Override
    public TensorFlow.goldMineral getMineral() throws InterruptedException
    {
        lookForMinerals();
        telemetry.addData("Gold", "%s position", location);
        telemetry.update();
        return location;
    }
}