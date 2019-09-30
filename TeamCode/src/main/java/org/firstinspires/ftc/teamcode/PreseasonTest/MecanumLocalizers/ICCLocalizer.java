package org.firstinspires.ftc.teamcode.PreseasonTest.MecanumLocalizers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Util;

import org.openftc.revextensions2.RevBulkData;

// Motivated by http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
public class ICCLocalizer implements BulkReadConsumer{

    MA3Encoder leftTrackingWheel = new MA3Encoder(0);
    MA3Encoder rightTrackingWheel = new MA3Encoder(1);
    MA3Encoder horizontalTrackingWheel = new MA3Encoder(2);



    public void init(BulkReadManager bulkReadManager) {
        bulkReadManager.registerHighPriority(leftTrackingWheel);
        bulkReadManager.registerHighPriority(rightTrackingWheel);
        bulkReadManager.registerHighPriority(horizontalTrackingWheel);
    }

    @Override
    public void bulkReadUpdate(RevBulkData revBulkData) {
        
    }
}
