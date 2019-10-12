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


        VuforiaLocalizer vuforia = initVuforia();

        Robot robot = new Robot(this.hardwareMap, this.telemetry, this);

        waitForStart();

        robot.changeRunModeToUsingEncoder();
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();

        telemetry.addLine("Got into runopmode");

        robot.moveToPoint(8,0,1,1,Math.toRadians(0));
        robot.finalTurn(0);

        telemetry.addLine("done with move");
        telemetry.update();

        telemetry.addLine("detecting vuforia");
        int position = robot.detectVuforia(vuforia);

        telemetry.addLine("go to point");
        if (position==1) {
            robot.moveToPoint(15, -4, 0.5, 0.5, Math.toRadians(0));
        } else if (position == 2){
            robot.moveToPoint(15,0,0.5,0.5,Math.toRadians(0));
        } else if (position == 3) {
            robot.moveToPoint(15,4,0.5,0.5,Math.toRadians(0));
        }

        telemetry.addLine("go back");
        robot.moveToPoint(-6,0,0.5,0.5, Math.toRadians(180));
        robot.finalTurn(Math.toRadians(90));

        telemetry.addLine("go under lander");
        PathPoints goUnderLander = new PathPoints(underLander, 9);
        robot.moveFollowCurve(goUnderLander.targetPoints);
    }

    protected VuforiaLocalizer initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters paramaters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        paramaters.vuforiaLicenseKey = VUFORIA_KEY;
        paramaters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        return ClassFactory.getInstance().createVuforia(paramaters);
    }
}