package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
        OpenGLMatrix latestLocationBackPerimeter = vSensor.listenerBackPerimeter.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationBluePerimeter = vSensor.listenerBluePerimeter.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationRedPerimeter = vSensor.listenerRedPerimeter.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationFrontPerimeter = vSensor.listenerFrontPerimeter.getUpdatedRobotLocation();

        // The listener will sometimes return null, so we check for that to prevent errors
        if (latestLocationBackPerimeter != null)
            vSensor.lastKnownLocationBackPerimeter = latestLocationBackPerimeter;
        if (latestLocationBluePerimeter != null)
            vSensor.lastKnownLocationBluePerimeter = latestLocationBluePerimeter;
        if (latestLocationRedPerimeter != null)
            vSensor.lastKnownLocationRedPerimeter = latestLocationRedPerimeter;
        if (latestLocationFrontPerimeter != null)
            vSensor.lastKnownLocationFrontPerimeter = latestLocationFrontPerimeter;

        // Send information about whether the target is visible, and where the robot is
        if (vSensor.listenerFrontPerimeter.isVisible())
            telemetry.addData("Tracking", vSensor.targetFrontPerimeter.getName());
        if (vSensor.listenerRedPerimeter.isVisible())
            telemetry.addData("Tracking", vSensor.targetRedPerimeter.getName());
        if (vSensor.listenerBluePerimeter.isVisible())
            telemetry.addData("Tracking", vSensor.targetBluePerimeter.getName());
        if (vSensor.listenerBackPerimeter.isVisible())
            telemetry.addData("Tracking", vSensor.targetBackPerimeter.getName());
    }

/////////////////////// VUFORIA ////////////////////////////////////////////////////////////////////

}