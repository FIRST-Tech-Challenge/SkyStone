package org.firstinspires.ftc.teamcode.Skystone.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;
import org.firstinspires.ftc.teamcode.Skystone.Detection;
import org.firstinspires.ftc.teamcode.Skystone.Tensorflow;

@Autonomous(name="TestVuforia")
public class TestVuforia extends AutoBase {
    final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";


    @Override
    public void runOpMode() {

        initLogic();
        Tensorflow tensorflow = new Tensorflow(robot);

        tensorflow.initTensorflow();
        waitForStart();


        while (opModeIsActive()){
            //try {
            //robot.moveToPoint(10,0,1,0,Math.toRadians(0));
            Detection tfodPosition = tensorflow.detectTensorflow();
            telemetry.addLine("Position: " +  tfodPosition.getPosition());
            telemetry.addLine("Value: " + tfodPosition.getValue());
            telemetry.addLine("numDetections: " + tfodPosition.getNumDetections());
            telemetry.update();
            //} catch (VuforiaException exception){
//                telemetry.addLine("Wack");
//                telemetry.update();
            //}
            //goToSkystone(tfodPosition, 0);
        }
    }
}
