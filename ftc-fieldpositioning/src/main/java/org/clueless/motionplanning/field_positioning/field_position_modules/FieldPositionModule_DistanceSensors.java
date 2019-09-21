package org.clueless.motionplanning.field_positioning.field_position_modules;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.clueless.motionplanning.field_positioning.FieldPositionModule;
import org.clueless.motionplanning.field_positioning.math.TwoDimensionalTransform;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import java.util.List;

public class FieldPositionModule_DistanceSensors extends FieldPositionModule{

    List<DistanceSensor> distanceSensors;
    List<OpenGLMatrix> distanceSensorPositions;

    @Override
    public TwoDimensionalTransform Update() {
        // 1. Only use if orientation is 90 degrees
        

        return null;
    }
}
