package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

@Autonomous(name="TestVuforia")
public class TestVuforia extends AutoBase {
    private static final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";

    @Override
    public void runOpMode() {
        robot.GoToVuforia();
    }
}