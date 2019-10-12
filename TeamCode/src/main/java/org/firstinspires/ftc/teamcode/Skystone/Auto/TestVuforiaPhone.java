package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.Skystone.Robot;


@Autonomous(name="TestVuforiaPhone")
public class TestVuforiaPhone extends AutoBase {
    private static final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";
    @Override
    public void runOpMode() {


        VuforiaLocalizer vuforia = robot.initVuforia(VUFORIA_KEY);

        Robot robot = new Robot(this.hardwareMap, this.telemetry, this);

        waitForStart();

        telemetry.addData("Vuforia Position: 1:Left, 2:Right, 3:Center: ", robot.detectVuforia(vuforia));
    }
}
