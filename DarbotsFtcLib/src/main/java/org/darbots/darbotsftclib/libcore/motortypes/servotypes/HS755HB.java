package org.darbots.darbotsftclib.libcore.motortypes.servotypes;

import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

public class HS755HB extends ServoType {
    //https://www.servocity.com/hs-755hb-servo
    @Override
    public String getServoName() {
        return "HS-755hb";
    }

    @Override
    public int getMaxDeg() {
        return 202;
    }

    @Override
    public int getMinDeg() {
        return 0;
    }

    @Override
    public double getRevPerSec() {
        return 60.0 / 0.28 / 202;
        //60 deg in 0.28s => 4.8V
        //60 deg in 0.23s => 6.0V
    }
}
