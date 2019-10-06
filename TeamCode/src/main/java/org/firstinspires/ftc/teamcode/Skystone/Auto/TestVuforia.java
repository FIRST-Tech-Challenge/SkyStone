package org.firstinspires.ftc.teamcode.Skystone.Auto;

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

    private VuforiaLocalizer vuforia = null;

    OpenGLMatrix lastLocation;
    int position = 0;
    private static final float mmPerInch = 25.4f;

    Point point = new Point();

    @Override
    public void runOpMode() {

        initVuforia();
        Robot robot = new Robot(this.hardwareMap, this.telemetry, this);

        waitForStart();

        robot.changeRunModeToUsingEncoder();
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();

        point.x = 17;
        point.y = 0;
        robot.moveToPoint(4,0,0.5,0.5,Math.toRadians(0));

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable skyStoneTarget = targetsSkyStone.get(0);

        targetsSkyStone.activate();
        boolean detected = false;
        int num = 0;
        while (!detected){
            if (((VuforiaTrackableDefaultListener) skyStoneTarget.getListener()).isVisible()) {

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) skyStoneTarget.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                VectorF translation = lastLocation.getTranslation();

                if (translation.get(0) / mmPerInch > 5) {
                    telemetry.addData("Position: ", "Left");
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//                double[][]testPoints={{6, -3},{17, -3}};
                    telemetry.update();
//               targetPoints = testPoints;
                    point.x = 9;
                    point.y = -3;
                    detected = true;

                } else if (translation.get(0) / mmPerInch < -5) {
                    telemetry.addData("Position: ", "Right");
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//                telemetry.update();
//                double[][]testPoints = {{6, 3},{17, 3}};

                    point.x = 9;
                    point.y = 3;
                    detected = true;

                } else {
                    telemetry.addData("Position: ", "Center");
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                    telemetry.update();
                    point.x = 9;
                    point.y = 0;
                    detected = true;
//               double[][]testPoints = {{6, 0},{17, 0}};
//               targetPoints = testPoints;
                }
                num++;
                if (num>100){
                    point.x = 9;
                    point.y = 0;
                    detected = true;
                    telemetry.addData("No Detect: ", "Defaulted");
                }
            }
        }

        sleep(10000);

        robot.moveToPoint(point.x,point.y,0.5,0.5,Math.toRadians(0));
        robot.finalTurn(0);

    }

    protected void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters paramaters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        paramaters.vuforiaLicenseKey = VUFORIA_KEY;
        paramaters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(paramaters);
    }
}