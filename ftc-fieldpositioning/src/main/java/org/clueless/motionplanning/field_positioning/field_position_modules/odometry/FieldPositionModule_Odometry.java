package org.clueless.motionplanning.field_positioning.field_position_modules.odometry;

import org.clueless.motionplanning.field_positioning.FieldPositionModule;


//

public  abstract class FieldPositionModule_Odometry extends FieldPositionModule {

    OdometryWheelProperties wheelProperties;

    public class OdometryWheelProperties {
        int encodersPerRevolution;
        double wheelDiameter; // In millimeters
    }

    public void setWheelProperties(OdometryWheelProperties wheelProperties) {
        this.wheelProperties = wheelProperties;
    }


}
