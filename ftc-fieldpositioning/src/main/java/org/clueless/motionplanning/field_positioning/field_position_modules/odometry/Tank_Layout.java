package org.clueless.motionplanning.field_positioning.field_position_modules.odometry;

import org.clueless.motionplanning.field_positioning.field_position_modules.odometry.encoders.Encoder;
import org.clueless.motionplanning.field_positioning.math.TwoDimensionalTransform;
public class Tank_Layout extends FieldPositionModule_Odometry {

    Encoder left;
    Encoder right;

    static double encoderDifferenceRotationFactor;

    public void setEncoderDifferenceRotationFactor(double factor) {
        encoderDifferenceRotationFactor = factor;
    }

    @Override
    public TwoDimensionalTransform Update() {
        int deltaLeft = left.deltaPosition();
        int deltaRight = right.deltaPosition();

        int averageForward = (deltaLeft + deltaRight) / 2;

        double deltaY = (averageForward / wheelProperties.encodersPerRevolution) * wheelProperties.wheelDiameter;

        double angleChange = (deltaLeft - deltaRight) * encoderDifferenceRotationFactor;

        AddVector(0, deltaY, angleChange);

        return transform;
    }
}
