package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
public class DashExample extends LinearOpMode {
    public static double someDouble = 0.0;
    public static int someInt = 0;
    public static boolean someBoolean = false;

    @Override
    public void runOpMode() throws InterruptedException {
        while(!isStopRequested()){
            TelemetryPacket packet = new TelemetryPacket();
            packet.addLine("sno");
            packet.put("magic", 245);
            packet.put("automated", 11115);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}