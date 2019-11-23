package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class TensorFlowBot extends PinchArmBot {
    public TensorFlowBot(LinearOpMode opMode) {
        super(opMode);
    }

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

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
    private static final String VUFORIA_KEY =
            "AW3DaKr/////AAABmbYMj0zPp0oqll2pQvI8zaoN8ktPz319ETtFtBMP7b609q4wWm6yRX9OVwWnf+mXPgSC/fSdDI2uUp/69KTNAJ6Kz+sTx+9DG+mymW00Xm3LP7Xe526NP/lM1CIBsOZ2DJlQ2mqmObbDs5WR5HXyfopN12irAile/dEYkr3uIFnJ95P19NMdbiSlNQS6SNzooW0Nc8cBKWz91P020YDqC4dHSpbQvYeFgVp2VWZJC/uyvmE15nePzZ30Uq/n8pIeYWKh4+XR74RoRyabXMXFB6PZz7lgKdRMhhhBvQ5Eh21VxjE5h8ZhGw27K56XDPk63eczGTYP/FfeLvTuK4iKSNyqRLS/37kuxKn3t/dlkwv1";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(ahwMap);
        } else {
            print("Sorry! This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }


    }

    public void driveUntilSeeSkystone(double maxPower, boolean forward){

        double power;

        if (forward){
            power = maxPower;
        }
        else{
            power = - maxPower;
        }

//        leftFront.setDirection(direction);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        while (this.opMode.opModeIsActive() && ! isSkystoneDetected()) {
            // Display it for the driver.
            print(String.format("Target : leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(),
                    leftRear.getCurrentPosition(),
                    rightRear.getCurrentPosition()));
        }
        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        //opMode.sleep(30000);
        print(String.format("Arrive @ leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition()));
    }

    @Override
    public void print(String message) {

    }

    public Boolean isSkystoneDetected(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());

                //for (Recognition recognition : updatedRecognitions) {
                //    if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                //        skystone1X = (int) recognition.getLeft();
                //    } else if (silverMineral1X == -1) {
                //        skystone2X = (int) recognition.getLeft();
                //    } else {
                //        silverMineral2X = (int) recognition.getLeft();
                //    }
                //}

                // step through the list of recognitions and display boundary info.
                int i = 0;
                Boolean found = false;
                for (Recognition recognition : updatedRecognitions) {
                    opMode.telemetry.log().add(String.format("label (%d)", i), recognition.getLabel());
                    opMode.telemetry.log().add(String.format("  left,top %.03f , %.03f",
                            recognition.getLeft(), recognition.getTop()));
                    opMode.telemetry.log().add(String.format("  right,bottom %.03f , %.03f",
                            recognition.getRight(), recognition.getBottom()));
                    RobotLog.d(String.format("label (%d)", i), recognition.getLabel());
                    RobotLog.d(String.format("  left,top %.03f , %.03f",
                            recognition.getLeft(), recognition.getTop()));
                    RobotLog.d(String.format("  right,bottom %.03f , %.03f",
                            recognition.getRight(), recognition.getBottom()));
                    found = recognition.getLabel() == LABEL_SECOND_ELEMENT;
                    if (found){
                        break;
                    }
                }
                opMode.telemetry.update();
                return found;
            }
            else{
                opMode.telemetry.addData("Tfod", "No objects found");
                opMode.telemetry.update();
                return false;
            }
        }
        else{
            opMode.telemetry.addData("Tfod", "not ready");
            opMode.telemetry.update();
            return true;

        }
    }
}
