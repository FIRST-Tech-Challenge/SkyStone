package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Blinker;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BBBlinkin {

    private RevBlinkinLedDriver _Blinkin;
    private Telemetry _telemetry;

    public void init(HardwareMap hwmap, Telemetry tele)
    {
        _Blinkin = hwmap.get(RevBlinkinLedDriver.class,"Blinkin");
        _telemetry = tele;
    }

    public void blinkOn()
    {
        _telemetry.addData("Turning On","");
        _Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
    }

    public void blinkOff()
    {
        _telemetry.addData("Turning Off","");
        _Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        //_Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.);
    }

    public void TimePeriod(int period)
    {
        if(period == 0)
        {
            _Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }

        else if(period == 1)
        {
            _Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }

        else if(period == 2)
        {
            _Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }

        else if(period == 3)
        {
            _Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        }

        else if(period == 4)
        {
            _Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }

        else
        {
            _Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }

}

