package org.firstinspires.ftc.teamcode.Skystone.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Vision;

@Autonomous(name="TestVuforia")
public class TestVuforia extends AutoBase {
    final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";


    @Override
    public void runOpMode() {

        initLogic();
        Vision tensorflow = new Vision(robot);

        tensorflow.initVision();
        waitForStart();

        //robot.moveToPoint(8.5 ,0,0.5,1,Math.toRadians(0));

        Vision.Location position = tensorflow.runDetection();
        double firstSkyStoneY = 0.0;
        double secondSkyStoneY = 24.0;


        if (position == Vision.Location.RIGHT){
            //telemetry.addLine("Position: Right");
            telemetry.update();
//            firstSkyStoneY = 3.0;
//            secondSkyStoneY = -12.0;
//            robot.moveToPoint(55,firstSkyStoneY,0.5,0.5,0);
        } else if (position == Vision.Location.CENTER){
            //telemetry.addLine("Position: Center");
            telemetry.update();
//            firstSkyStoneY = 0.0;
//            secondSkyStoneY = -15.0;
//            robot.moveToPoint(55, firstSkyStoneY, 0.5, 0.5,0);
        } else {
            //telemetry.addLine("Position: Center");
            telemetry.update();
//            firstSkyStoneY = -3.0;
//            secondSkyStoneY = -35.0;
//            robot.moveToPoint(55,firstSkyStoneY,0.5,0.5,0);
        }
        sleep(99999999);
    }
}
