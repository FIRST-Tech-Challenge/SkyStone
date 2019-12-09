package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;

import java.util.function.Supplier;

public class VuforiaNavigator {
    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private HardwareChassis robot;
    private HardwareMap hardwareMap;
    private VuforiaTrackable stoneTarget;
    private ControlledDrive controlledDrive;
    private Telemetry telemetry;

    double targetX;
    double targetY;
    double targetZ;
    double rX;
    double rY;
    double rZ;

    public VuforiaNavigator(HardwareMap hardwareMap, HardwareChassis robot, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        this.controlledDrive = new ControlledDrive(hardwareMap);
        this.telemetry = telemetry;

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         */
        int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

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
        parameters.vuforiaLicenseKey =
                "ATuf55z/////AAAAGZJ5ehWMTUlerf1dXYv+I+OJqTIps4ZDcfS334fqN+YmRk5L9hG8TkFrH2XOu2xaZeIl1iTY9PYO0W17E3Ug9hyK/s6I6soSKV+vjdQN2YnXnprec+wSgxSFqPHN9lxRGyD7D5xqMtNXM2inPsxpzvfQGuAOouGHWoyKkfqSJGywlH/9e+EDBJVXZ2Gn0OUmE/Ee0pS70SPan6WR7KZY+/UbZkVgGNai9neL6g3N9/8/EJ1ln/mZj3Wti4Ntzo/+ZsnW6dVlrXGvBPZ4+nOmTL+4YXVUZPBdL5HXlBZBR3hfyOg3Ovlqcr08oC2VvKjLVIXK4/Ab9FsNqb3yjCudVKPQh0zBsOS2+x7+ZxYLSDtc";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        /**
         * Instantiate the Vuforia engine
         */
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        this.stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        targetsSkyStone.activate();
    }

    /**
     * is skystone target found
     * @return boolean
     */
    public boolean skystoneFound() {
        return ((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible();
    }

    public void navigateToSkystone(double forwardSpeed, double sidewardsSpeed) {
        double[] result = OmniWheel.calculate(5.0, 38, 24, 0, sidewardsSpeed, 0);
        robot.motor_front_left.setPower(result[0]);
        robot.motor_front_right.setPower(result[1]);
        robot.motor_rear_left.setPower(result[2]);
        robot.motor_rear_right.setPower(result[3]);
        telemetry.addData("State", "sidewards till transl/rrient");
        telemetry.update();

        while (getTranslationAndOrientation()[1] != 0) {}

        result = OmniWheel.calculate(5.0, 38, 24, forwardSpeed, 0, 0);
        robot.motor_front_left.setPower(result[0]);
        robot.motor_front_right.setPower(result[1]);
        robot.motor_rear_left.setPower(result[2]);
        robot.motor_rear_right.setPower(result[3]);
        telemetry.addData("State", "forward");
        telemetry.update();

        while (getTranslationAndOrientation()[0] != 3) {}

        robot.motor_front_left.setPower(0);
        robot.motor_front_right.setPower(0);
        robot.motor_rear_left.setPower(0);
        robot.motor_rear_right.setPower(0);

        //controlledDrive.driveConditionally(0, speed, () -> getTranslationAndOrientation()[1] != 0);
        //controlledDrive.driveConditionally(speed, 0, () -> getTranslationAndOrientation()[0] != 3);
    }

    public double[] getTranslationAndOrientation() {
        /**
         * See if any of the instances of {@link targetsSkyStone} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */

        if (((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()) {

            /* Found an instance of the template. In the actual game, you will probably
             * loop until this condition occurs, then move on to act accordingly depending
             * on which VuMark was visible. */
            /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
             * it is perhaps unlikely that you will actually need to act on this pose information, but
             * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)stoneTarget.getListener()).getPose();

            /* We further illustrate how to decompose the pose into useful rotational and
             * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                targetX = trans.get(0);
                targetY = trans.get(1);
                targetZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                rX = rot.firstAngle;
                rY = rot.secondAngle;
                rZ = rot.thirdAngle;

                double[] ret_array = {targetX, targetY, targetZ, rX, rY, rZ};
                return ret_array;
            }
        }
        double[] ret_array = {0,0,0,0,0,0};
        return ret_array;
    }

    //Methods to show Target X, Y, Z in telemetry
    public int showTargetX() {
        return (int) targetX;
    }
    public int showTargetY() {
        return (int) targetY;
    }
    public int showTargetZ() {
        return (int) targetZ;
    }

    //Methods to return angle to target
    public int showRX(){
        return (int) rX;
    }
    public int showRY(){
        return (int) rY;
    }
    public int showRZ(){
        return (int) rZ;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
