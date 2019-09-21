package org.clueless.motionplanning.field_positioning;

import org.clueless.motionplanning.field_positioning.field_position_modules.FieldPositionModule_DistanceSensors;
import org.clueless.motionplanning.field_positioning.field_position_modules.FieldPositionModule_Vuforia;
import org.clueless.motionplanning.field_positioning.math.TwoDimensionalTransform;
import org.clueless.motionplanning.field_positioning.math.Vector2;

import java.util.ArrayList;
import java.util.List;

public class FieldPositioningManager {
    List<FieldPositionModule> modules;

    public void Initialize() {
        modules = new ArrayList<>();
        modules.add(new FieldPositionModule_DistanceSensors());
        modules.add(new FieldPositionModule_Vuforia());

    }

    public TwoDimensionalTransform Update() {
        List<TwoDimensionalTransform> transforms = new ArrayList<>();

        for (FieldPositionModule m:
             modules) {
            if (m.isActive) {
                transforms.add(m.Update());
            }
        }

        return calculateAverage(transforms);
    }

    private TwoDimensionalTransform calculateAverage(List<TwoDimensionalTransform> transformsList) {
        double x = 0;
        double y = 0;
        double angle = 0;
        if(!transformsList.isEmpty()) {
            for (TwoDimensionalTransform mark : transformsList) {
                x += mark.vector.x;
                y += mark.vector.y;
                angle += mark.angle;
            }
        }
        return new TwoDimensionalTransform(new Vector2(x, y), angle);
    }
}
