package org.firstinspires.ftc.teamcode.SubAssembly.FoundationGrabber;

import android.app.Notification;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.EnumWrapper;
import org.firstinspires.ftc.teamcode.Utilities.ServoControl;

import java.util.EnumMap;

/* Sub Assembly Class
 */
public class FoundationGrabberControl {
    /* Declare private class object */
    private LinearOpMode opmode = null;     /* local copy of opmode class */

    //initializing motors
    private Servo foundationMover;


    public ServoControl<FoundationGrabberSetpt, EnumMap<FoundationGrabberSetpt, Double>> FoundationGrabberServo;


    // declare servo mapping variables
    private EnumMap<FoundationGrabberSetpt, Double> MapFoundationGrabber;


    public enum FoundationGrabberSetpt implements EnumWrapper<FoundationGrabberSetpt> {
        Close, Open;
    }


    /* Subassembly constructor */
    public FoundationGrabberControl() {
    }

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;

        opMode.telemetry.addLine("Foundation Grabber Control" + " initialize");
        opMode.telemetry.update();

        /* Set local copies from opmode class */
        opmode = opMode;
        hwMap = opMode.hardwareMap;

        // create servo mappings
       /* MapFoundationGrabber = new EnumMap<FoundationGrabberSetpt, Double>(FoundationGrabberSetpt.class);
        MapFoundationGrabber.put(FoundationGrabberSetpt.Close, 0.0);
        MapFoundationGrabber.put(FoundationGrabberSetpt.Open, 1.0);


        /* Map hardware devices */
        foundationMover = hwMap.servo.get("foundationMover");


      //  FoundationGrabberServo = new ServoControl(foundationMover, MapFoundationGrabber, FoundationGrabberSetpt.Open, true);

        foundationMover.setPosition(1.0);
    }

    public void close() {
        foundationMover.setPosition(0.0);
    }
    public void open(){
        foundationMover.setPosition(1.0);
    }

}
