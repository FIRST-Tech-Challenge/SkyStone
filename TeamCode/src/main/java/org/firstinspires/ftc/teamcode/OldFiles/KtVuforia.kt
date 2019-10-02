package org.firstinspires.ftc.teamcode.OldFiles


/**
 * Created by KasaiYuki on 1/15/2019.
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection
import org.firstinspires.ftc.robotcore.external.tfod.*


@TeleOp(name = "KtVuforia", group = "Autonomous")
@Disabled
class KtVuforia : LinearOpMode()
{
    val robot = KtRobot()
    private val TFOD_MODEL_ASSET = "RoverRuckus.tflite"
    private val LABEL_GOLD_MINERAL = "Gold Mineral"
    private val LABEL_SILVER_MINERAL = "Silver Mineral"
    var lSize: Int = 0
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     * ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private val VUFORIA_KEY = "AYCFV6H/////AAAAmekk6YIrl0eXiv6v/NSJkjV9eTSHNlS21VtbOvMpT/DU+eSVKRogBQ23cQ+qHItgdLdAyG0XIKpVWBewwHog581BgoBEyhLGzhnxI/57o+CYFi372QAb8faGRN/tE22Pm9BTinccijuCDITIS/9W4mQeUOGOMIC5rB76NZPeNT20Oj65AaG2s5N90hvh2+5xeQ4nhW3w34eez9C3tmO8A9ErqPG+CfDgKPhGZmI7SkAGvUlfzQDFvxPNeK8nYpD3ZnBYq+jytcTR5ch9MjrE0Oqbp5m+RnUIDNC7fP/4JPZ8l5i4JP6dvF1MAhpeJcAU2dIP7umddnO1M/mOOCZNwBD1o1qUMWjXSkvtBFtTstNl"
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private var vuforia: VuforiaLocalizer? = null
    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private var tfod: TFObjectDetector? = null
    override fun runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia()
        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod()
        }
        else
        {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD")
        }
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking")
        telemetry.update()
        waitForStart()
        if (opModeIsActive())
        {
            /** Activate Tensor Flow Object Detection. */
            tfod?.activate()//if tfod is not null, activate
            while (opModeIsActive())
            {
                if (tfod != null)
                {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    val updatedRecognitions: List<Recognition> = tfod!!.updatedRecognitions
                    if (updatedRecognitions != null)
                    {
                        lSize = updatedRecognitions.size
                        telemetry.addData("# Object Detected", "(int) detected", lSize)
                        if (lSize == 3)
                        {
                            var goldMineralX = -1
                            var silverMineral1X = -1
                            var silverMineral2X = -1
                            for (recognition in updatedRecognitions)
                            {
                                when {
                                    recognition.label == LABEL_GOLD_MINERAL -> goldMineralX = recognition.left.toInt()
                                    silverMineral1X == -1 -> silverMineral1X = recognition.left.toInt()
                                    else -> silverMineral2X = recognition.left.toInt()
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1)
                            {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X)
                                {
                                    telemetry.addData("Gold Mineral Position", "Left")
                                    robot.getCube("left")
                                }
                                else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)
                                {
                                    telemetry.addData("Gold Mineral Position", "Right")
                                    robot.getCube("right")
                                }
                                else
                                {
                                    telemetry.addData("Gold Mineral Position", "Center")
                                    robot.getCube("center")
                                }
                            }
                        }
                        telemetry.update()
                    }
                }
            }
        }
        if (tfod != null)
        {
            tfod?.shutdown()
        }
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private fun initVuforia() {
        /*
       * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
       */
        val parameters = VuforiaLocalizer.Parameters()
        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraDirection = CameraDirection.BACK
        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters)
        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private fun initTfod() {
        val tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        val tfodParameters = TFObjectDetector.Parameters(tfodMonitorViewId)
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)
        tfod?.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL)
    }
}
