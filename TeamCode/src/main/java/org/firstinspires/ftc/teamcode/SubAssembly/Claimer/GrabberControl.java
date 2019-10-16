package org.firstinspires.ftc.teamcode.SubAssembly.Claimer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* Sub Assembly Class
 */
public class GrabberControl {
    /* Declare private class object */
    private LinearOpMode opmode = null;     /* local copy of opmode class */

    //initializing motors
    private Servo grabberS;
    private Servo wristS;

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

        /* Map hardware devices */
        grabberS = hwMap.servo.get("grabberS");
        wristS = hwMap.servo.get("wristS");

        grabberS.setPosition(0);
        wristS.setPosition(0);

    }

    public void open() {
        grabberS.setPosition(0.92);
    }

    public void close() {
        grabberS.setPosition(0);
    }

    public void wrist(int position) {
        wristS.setPosition(position);
    }

}
