package org.firstinspires.ftc.teamcode.auto;


        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


    public class VuforiaFindBlock extends LinearOpMode {
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();

        private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
        private static final boolean PHONE_IS_PORTRAIT = false;

        private static final String VUFORIA_KEY =
                "ARu5ZM3/////AAABmcGiY9yUyEAsnf3dcn+gux+E9X/ji5wR1QEra3aJBAbIFoL8BPmzx+eUt8sZ7bEwE4IRvwNm32oB/EDVFrGwZtkyOiSR+GKIbM+0G5VYQGwoFNxxGwuUrvpKDS3ktLAuUWmZ0/p/f7ZGwr9di1s4JkzDwr9Hq2B1g16a5F2jf7te3PhLDYaeauXee+WNxv0hp2w64Q91mYwiI+dI9JKsvyruF/FVKVV5Dnf0IGn9mFDGhqSGfkXTOGNpBnjZes5rxndN0PVhvJD+nf1ohsL37m8ORe9zXJqAUJ+vBCaJn7tCtsBJpKgBXYLpWrm05PVXex5cGQsgNc++80BpymExMMCpi6woERNjR86v/cyL+gqg";
        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
        private static final float mmPerInch = 25.4f;
        private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

        // Constant for Stone Target
        private static final float stoneZ = 2.00f * mmPerInch;

        // Constants for the center support targets
        private static final float bridgeZ = 6.42f * mmPerInch;
        private static final float bridgeY = 23 * mmPerInch;
        private static final float bridgeX = 5.18f * mmPerInch;
        private static final float bridgeRotY = 59;                                 // Units are degrees
        private static final float bridgeRotZ = 180;

        // Constants for perimeter targets
        private static final float halfField = 72 * mmPerInch;
        private static final float quadField = 36 * mmPerInch;

        // Class Members
        private OpenGLMatrix lastLocation = null;
        private VuforiaLocalizer vuforia = null;
        private boolean targetVisible = false;
        private float phoneXRotate = 0;
        private float phoneYRotate = 0;
        private float phoneZRotate = 0;

        int x, y;

        public void runOpMode() throws InterruptedException {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = CAMERA_CHOICE;

            vuforia = ClassFactory.getInstance().createVuforia(parameters);
            //FtcDashboard.getInstance().startCameraStream(vuforia, 0);

            //FtcDashboard dashboard = FtcDashboard.getInstance();
            //Telemetry dashboardTelemetry = dashboard.getTelemetry();

            VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

            VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");
            VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
            blueRearBridge.setName("Blue Rear Bridge");
            VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
            redRearBridge.setName("Red Rear Bridge");
            VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
            redFrontBridge.setName("Red Front Bridge");
            VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
            blueFrontBridge.setName("Blue Front Bridge");
            VuforiaTrackable red1 = targetsSkyStone.get(5);
            red1.setName("Red Perimeter 1");
            VuforiaTrackable red2 = targetsSkyStone.get(6);
            red2.setName("Red Perimeter 2");
            VuforiaTrackable front1 = targetsSkyStone.get(7);
            front1.setName("Front Perimeter 1");
            VuforiaTrackable front2 = targetsSkyStone.get(8);
            front2.setName("Front Perimeter 2");
            VuforiaTrackable blue1 = targetsSkyStone.get(9);
            blue1.setName("Blue Perimeter 1");
            VuforiaTrackable blue2 = targetsSkyStone.get(10);
            blue2.setName("Blue Perimeter 2");
            VuforiaTrackable rear1 = targetsSkyStone.get(11);
            rear1.setName("Rear Perimeter 1");
            VuforiaTrackable rear2 = targetsSkyStone.get(12);
            rear2.setName("Rear Perimeter 2");

            List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(targetsSkyStone);

            stoneTarget.setLocation(OpenGLMatrix
                    .translation(0, 0, stoneZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


            blueFrontBridge.setLocation(OpenGLMatrix
                    .translation(-bridgeX, bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

            blueRearBridge.setLocation(OpenGLMatrix
                    .translation(-bridgeX, bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

            redFrontBridge.setLocation(OpenGLMatrix
                    .translation(-bridgeX, -bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

            redRearBridge.setLocation(OpenGLMatrix
                    .translation(bridgeX, -bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

            red1.setLocation(OpenGLMatrix
                    .translation(quadField, -halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            red2.setLocation(OpenGLMatrix
                    .translation(-quadField, -halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            front1.setLocation(OpenGLMatrix
                    .translation(-halfField, -quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

            front2.setLocation(OpenGLMatrix
                    .translation(-halfField, quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

            blue1.setLocation(OpenGLMatrix
                    .translation(-quadField, halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

            blue2.setLocation(OpenGLMatrix
                    .translation(quadField, halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

            rear1.setLocation(OpenGLMatrix
                    .translation(halfField, quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            rear2.setLocation(OpenGLMatrix
                    .translation(halfField, -quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            if (CAMERA_CHOICE == BACK) {
                phoneYRotate = -90;
            } else {
                phoneYRotate = 90;
            }

            if (PHONE_IS_PORTRAIT) {
                phoneXRotate = 90;
            }

            // camera lens, where it is on the robot. robot 16.75 long 17.5 wide
            final float CAMERA_FORWARD_DISPLACEMENT = 6.75f * mmPerInch;   // eg: Camera is 7 Inches in front of robot center
            final float CAMERA_VERTICAL_DISPLACEMENT = 11.0f * mmPerInch;   // eg: Camera is 11 Inches above ground
            final float CAMERA_LEFT_DISPLACEMENT = -1.875f * mmPerInch;     // eg: Camera is 2.25 Inches left of robot center

            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            /**  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            }

            targetsSkyStone.activate();
            telemetry.addData("target detected?", targetVisible);
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            //////////////////////////////////////////////////////////////////////////


            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {

                /*robot.auto_forward(850, 1);//please
                //robot.auto_turnright(250,1);//please
                while (opModeIsActive() && robot.encoderIsBusy()) {
                    idle();
                }
                sleep(1000);

                robot.stop();*/

                //////////////////////////////


                /*while (!isVisible(allTrackables)) {
                    robot.runwithoutencoder();
                    robot.withoutencoder_strafe_left(0.25);
                    Thread.sleep(1000);
                    robot.stop();
                    telemetry.addData("target detected?", targetVisible);
                    telemetry.update();
                    Thread.sleep(400);

                }

                telemetry.addData("target detected?", targetVisible);

                robot.stop();

                telemetry.update();

                robot.resetEncoder();*/

                ////////////////

                for (VuforiaTrackable trackable : allTrackables) {
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                        break;
                    }
                }

                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                x = (int) (translation.get(0) / mmPerInch);
                y = (int) (translation.get(1) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);


                ////////////////////

            /*robot.auto_strafeleft(35 * x, 0.25); //35 is ticks per inch
            while (opModeIsActive() && robot.encoderIsBusy()) {
                idle();
                telemetry.addData("run to pos", 35*x);
                telemetry.update();

            }*/
                sleep(1000);

                targetsSkyStone.deactivate();
                stop();
            }

        }








        public boolean isVisible(List<VuforiaTrackable> allTrackables) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                }
            }
            return targetVisible;
        }
    }
