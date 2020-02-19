package org.firstinspires.ftc.teamcode.SubAssembly.Grabber;

import android.app.Notification;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.EnumWrapper;
import org.firstinspires.ftc.teamcode.Utilities.ServoControl;

import java.util.EnumMap;

/* Sub Assembly Class
 */
public class GrabberControl {
    /* Declare private class object */
    private LinearOpMode opmode = null;     /* local copy of opmode class */

    //initializing motors
    private Servo grabberS;
    private Servo wristS;
    private Servo extenderS;

    public ServoControl<GrabberSetpt, EnumMap<GrabberSetpt, Double>> GrabberServo;
    public ServoControl<WristSetpt, EnumMap<WristSetpt, Double>> WristServo;
    public ServoControl<ExtenderSetpt, EnumMap<ExtenderSetpt, Double>> ExtenderServo;

    // declare servo mapping variables
    private EnumMap<GrabberSetpt, Double> MapGrabber;
    private EnumMap<WristSetpt, Double> MapWrist;
    private EnumMap<ExtenderSetpt, Double> MapExtender;

    public enum GrabberSetpt implements EnumWrapper<GrabberSetpt> {
        Close, Open;
    }

    public enum WristSetpt implements EnumWrapper<WristSetpt> {
        Horizontal, Vertical;
    }

    public enum ExtenderSetpt implements EnumWrapper<ExtenderSetpt> {
        Home, Pos0, Pos1, Pos2, Pos3;
    }

    /* Subassembly constructor */
    public GrabberControl() {
    }

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;

        opMode.telemetry.addLine("Grabber Control" + " initialize");
        opMode.telemetry.update();

        /* Set local copies from opmode class */
        opmode = opMode;
        hwMap = opMode.hardwareMap;

        // create servo mappings
        MapGrabber = new EnumMap<GrabberSetpt, Double>(GrabberSetpt.class);
        MapGrabber.put(GrabberSetpt.Open, 0.00001);
        MapGrabber.put(GrabberSetpt.Close, 0.79);

        MapWrist = new EnumMap<WristSetpt, Double>(WristSetpt.class);
        MapWrist.put(WristSetpt.Horizontal, 0.44);
        MapWrist.put(WristSetpt.Vertical, 0.85);

        MapExtender = new EnumMap<ExtenderSetpt, Double>(ExtenderSetpt.class);
        MapExtender.put(ExtenderSetpt.Home, 0.81);
        MapExtender.put(ExtenderSetpt.Pos0, 0.70);
        MapExtender.put(ExtenderSetpt.Pos1, 0.52);
        MapExtender.put(ExtenderSetpt.Pos2, 0.30);
        MapExtender.put(ExtenderSetpt.Pos3, 0.13);

        /* Map hardware devices */
        grabberS = hwMap.servo.get("grabberS");
        wristS = hwMap.servo.get("wristS");
        extenderS = hwMap.servo.get("extenderS");

        GrabberServo = new ServoControl(grabberS, MapGrabber, GrabberSetpt.Close, true);
        WristServo = new ServoControl(wristS, MapWrist, WristSetpt.Horizontal, true);
        ExtenderServo = new ServoControl(extenderS, MapExtender, ExtenderSetpt.Home, true);

    }

    public void open() {
        GrabberServo.setSetpoint(GrabberSetpt.Open);
    }

    public void close() {
        GrabberServo.setSetpoint(GrabberSetpt.Close);
    }

    public void grab() {
        GrabberServo.nextSetpoint(true);
        /*if (GrabberServo.getSetpoint() == GrabberSetpt.Open)
            GrabberServo.setSetpoint(GrabberSetpt.Close);
        else
            GrabberServo.setSetpoint(GrabberSetpt.Open);*/
    }

    public void wrist() {
        WristServo.nextSetpoint(true);
    }

    public void extend() {
        ExtenderServo.nextSetpoint(false);
    }

    public void Pos0() {
        ExtenderServo.setSetpoint(ExtenderSetpt.Pos0);
    }

    public void Pos1() {
        ExtenderServo.setSetpoint(ExtenderSetpt.Pos1);
    }
}
