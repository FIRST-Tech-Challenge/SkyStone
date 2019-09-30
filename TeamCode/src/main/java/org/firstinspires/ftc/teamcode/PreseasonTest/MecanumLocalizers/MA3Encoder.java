package org.firstinspires.ftc.teamcode.PreseasonTest.MecanumLocalizers;

import org.openftc.revextensions2.RevBulkData;

public class MA3Encoder implements BulkReadConsumer {
    private int pin;

    public MA3Encoder(int pin) {
        this.pin = pin;
    }

    @Override
    public void bulkReadUpdate(RevBulkData revBulkData) {

    }
}
