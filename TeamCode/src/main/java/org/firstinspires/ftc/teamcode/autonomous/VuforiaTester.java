package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class VuforiaTester extends OpMode {

    VuforiaSensor vSensor = new VuforiaSensor();

    public VuforiaTester() {
        super();
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {
        vSensor.visionTargets.activate();
    }

    @Override
    public void loop() {
        vuforiaLoop();
    }

/////////////////////// VUFORIA ////////////////////////////////////////////////////////////////////

    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    private void vuforiaLoop() {
        // Ask the listener for the latest information on where the robot is
        OpenGLMatrix latestLocationSkystone = vSensor.listenerSkystone.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationRedPerimeterTgt1 = vSensor.listenerRedPerimeterTgt1.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationRedPerimeterTgt2 = vSensor.listenerRedPerimeterTgt2.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationFrontPerimeterTgt1 = vSensor.listenerFrontPerimeterTgt1.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationFrontPerimeterTgt2 = vSensor.listenerFrontPerimeterTgt2.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationBluePerimeterTgt1 = vSensor.listenerBluePerimeterTgt1.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationBluePerimeterTgt2 = vSensor.listenerBluePerimeterTgt2.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationRearPerimeterTgt1 = vSensor.listenerRearPerimeterTgt1.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationRearPerimeterTgt2 = vSensor.listenerRearPerimeterTgt2.getUpdatedRobotLocation();

        // The listener will sometimes return null, so we check for that to prevent errors
        if (latestLocationSkystone != null)
            vSensor.lastKnownLocationSkystone = latestLocationSkystone;
        if (latestLocationRedPerimeterTgt1 != null)
            vSensor.lastKnownLocationRedPerimeterTgt1 = latestLocationRedPerimeterTgt1;
        if (latestLocationRedPerimeterTgt2 != null)
            vSensor.lastKnownLocationRedPerimeterTgt2 = latestLocationRedPerimeterTgt2;
        if (latestLocationFrontPerimeterTgt1 != null)
            vSensor.lastKnownLocationFrontPerimeterTgt1 = latestLocationFrontPerimeterTgt1;
        if (latestLocationFrontPerimeterTgt2 != null)
            vSensor.lastKnownLocationFrontPerimeterTgt2 = latestLocationFrontPerimeterTgt2;
        if (latestLocationBluePerimeterTgt1 != null)
            vSensor.lastKnownLocationBluePerimeterTgt2 = latestLocationBluePerimeterTgt1;
        if (latestLocationBluePerimeterTgt2 != null)
            vSensor.lastKnownLocationBluePerimeterTgt2 = latestLocationBluePerimeterTgt2;
        if (latestLocationRearPerimeterTgt1 != null)
            vSensor.lastKnownLocationRearPerimeterTgt2 = latestLocationRearPerimeterTgt1;
        if (latestLocationRearPerimeterTgt2 != null)
            vSensor.lastKnownLocationRearPerimeterTgt2 = latestLocationRearPerimeterTgt2;

        // Send information about whether the target is visible, and where the robot is
        if (vSensor.listenerSkystone.isVisible())
            telemetry.addData("Tracking", vSensor.targetSkystone.getName());
        if (vSensor.listenerRedPerimeterTgt1.isVisible())
            telemetry.addData("Tracking", vSensor.targetRedPerimeterTgt1.getName());
        if (vSensor.listenerRedPerimeterTgt2.isVisible())
            telemetry.addData("Tracking", vSensor.targetRedPerimeterTgt2.getName());
        if (vSensor.listenerFrontPerimeterTgt1.isVisible())
            telemetry.addData("Tracking", vSensor.targetFrontPerimeterTgt1.getName());
        if (vSensor.listenerFrontPerimeterTgt2.isVisible())
            telemetry.addData("Tracking", vSensor.targetFrontPerimeterTgt2.getName());
        if (vSensor.listenerBluePerimeterTgt1.isVisible())
            telemetry.addData("Tracking", vSensor.targetBluePerimeterTgt1.getName());
        if (vSensor.listenerBluePerimeterTgt2.isVisible())
            telemetry.addData("Tracking", vSensor.targetBluePerimeterTgt2.getName());
        if (vSensor.listenerRearPerimeterTgt1.isVisible())
            telemetry.addData("Tracking", vSensor.targetRearPerimeterTgt1.getName());
        if (vSensor.listenerRearPerimeterTgt2.isVisible())
            telemetry.addData("Tracking", vSensor.targetRearPerimeterTgt2.getName());

    }

/////////////////////// VUFORIA ////////////////////////////////////////////////////////////////////

}