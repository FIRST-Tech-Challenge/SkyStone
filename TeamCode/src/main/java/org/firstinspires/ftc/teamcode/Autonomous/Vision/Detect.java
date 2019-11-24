package org.firstinspires.ftc.teamcode.Autonomous.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Tensorflow.TFODCalc;

import java.util.ArrayList;
import java.util.List;

public class Detect {
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public Detect(){
        TFODCalc.init();
        TFODCalc.setHardwareProperties(43.30, 3.67f);

        /*if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap, model, firstElement, secondElement);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }*/
    }

    public int[] getPositions(List<Recognition> updatedRecognitions){
        if (updatedRecognitions != null) {
            // step through the list of recognitions and display boundary info.
            int i = 0;
            int index = 0;
            double[] left = new double[updatedRecognitions.size()];
            ArrayList<String> skystoneIndex = new ArrayList<>();

            for (Recognition recognition : updatedRecognitions) {
                left[index] = recognition.getLeft();
                if(recognition.getLabel().equalsIgnoreCase("skystone"))
                    skystoneIndex.add("skystone");
                else
                    skystoneIndex.add("stone");
                index += 1;
            }

            switch(updatedRecognitions.size()){
                case 1:
                    return new int[] {0, 0};
                case 2:
                    if(!skystoneIndex.contains("skystone")) {
                        return new int[]{3, 6};
                    } else {
                        if(left[skystoneIndex.indexOf("skystone")] > left[skystoneIndex.indexOf("stone")])
                            return new int[] {2, 5};
                        else if(left[skystoneIndex.indexOf("skystone")] <= left[skystoneIndex.indexOf("stone")])
                            return new int[] {1, 4};
                    }
                    break;
            }

            if(skystoneIndex.contains("skystone")) {
                double minPos = 9999;
                if (updatedRecognitions.size() >= 3) {
                    for (int x = 0; x < skystoneIndex.size(); x++) {
                        if (skystoneIndex.get(x).equalsIgnoreCase("skystone")) {
                            if (minPos > updatedRecognitions.get(x).getLeft())
                                minPos = updatedRecognitions.get(x).getLeft();
                        }
                    }

                    int idx = 0;
                    for (Recognition r : updatedRecognitions) {
                        if (r.getLeft() < minPos)
                            idx += 1;
                    }
                    return new int[]{idx + 1, idx + 4};
                }
            }
        }// -1.5707 , -3.14
        return new int[] {0, 0};
    }

    public double getImageHeight() throws InterruptedException{
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue

        return frame.getImage(0).getHeight();
    }

    public VuforiaLocalizer getVuforia(){
        return vuforia;
    }

    public TFObjectDetector getTfod(){
        return tfod;
    }

    public List<Recognition> getUpdatedRecognitions(){
        return tfod.getUpdatedRecognitions();
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */

    private void initVuforia(HardwareMap hardwareMap){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";  //Variable Place--Remember to insert key here
        parameters.cameraName = hardwareMap.get(WebcamName.class, "WebcamFront");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod(HardwareMap hardwareMap,
                          String model, String firstElement, String secondElement)  {
        initVuforia(hardwareMap);

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.


        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.85;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(model, firstElement, secondElement);
    }
}
