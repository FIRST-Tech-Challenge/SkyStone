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

    public static double[][] underLander = {{8,0},{8,50},{20,50}};

    @Override
    public void runOpMode() {

        // init vuforia

        Robot robot = new Robot(this.hardwareMap, this.telemetry, this);

        waitForStart();

        robot.changeRunModeToUsingEncoder();

        // start odometry
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();
        robot.resetEncoders();
        robot.changeRunModeToUsingEncoder();
        telemetry.addLine("Got into runopmode");

        robot.moveToPoint(12 ,0,1,1,Math.toRadians(0));
        robot.absoluteTurn(0, 0.7);
        robot.absoluteTurn(Math.toRadians(0),0.5);

        telemetry.addLine("done with move");
        telemetry.update();

        telemetry.addLine("go to point");
        int position = 0;
        String str1ng = robot.detectTensorflow();
        robot.intakeRight.setPower(0.8);
        robot.intakeLeft.setPower(0.8);

        if (str1ng.equals("right")) {
            robot.moveToPoint(39, 9, 0.45, 0.5, Math.toRadians(0));
        } else if (str1ng.equals("left")){
            robot.moveToPoint(39, -9, 0.45, 0.5, Math.toRadians(0));
        } else {
            robot.moveToPoint(39, 0, 0.45, 0.5, Math.toRadians(0));
        }

//        telemetry.addData("Position: -1:Left, 0:Center, 1:Right", position);
//        telemetry.update();
//        telemetry.addLine("go back");
//        robot.moveToPoint(7,0,0.5,0.5, Math.toRadians(180));
//        robot.finalTurn(Math.toRadians(90));

//        telemetry.addLine("go under lander");
//        PathPoints goUnderLander = new PathPoints(underLander, 9);
//        robot.moveFollowCurve(goUnderLander.targetPoints);
    }
}