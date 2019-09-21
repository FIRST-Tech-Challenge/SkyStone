package org.clueless.motionplanning.field_positioning.field_position_modules.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.clueless.motionplanning.field_positioning.field_position_modules.odometry.encoders.Encoder;
import org.clueless.motionplanning.field_positioning.math.TwoDimensionalTransform;

public class XY_Layout extends FieldPositionModule_Odometry {

    Encoder forward;
    Encoder horizontal;

    BNO055IMU gyroscope;

    @Override
    public TwoDimensionalTransform Update() {
        int deltaHorizontal = horizontal.deltaPosition();
        int deltaForward = forward.deltaPosition();

        double deltaY = (deltaForward / wheelProperties.encodersPerRevolution) * wheelProperties.wheelDiameter;
        double deltaX = (deltaHorizontal / wheelProperties.encodersPerRevolution) * wheelProperties.wheelDiameter;

        double deltaAngle = gyroscope.getAngularOrientation().firstAngle - transform.angle;

        AddVector(deltaX, deltaY, deltaAngle);

        return transform;
    }
}
