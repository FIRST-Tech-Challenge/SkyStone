package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;

@Autonomous
public class TouchSensorCrater extends AutoOpMode {

    /**
     * The REV Robotics Touch Sensor
     * is treated as a digital channel.  It is HIGH if the button is unpressed.
     * It pulls LOW if the button is pressed.
     * <p>
     * Also, when you connect a REV Robotics Touch Sensor to the digital I/O port on the
     * Expansion Hub using a 4-wire JST cable, the second pin gets connected to the Touch Sensor.
     * The lower (first) pin stays unconnected.*
     */

    public DigitalChannel magnetSensor;  // Hardware Device Object

    @Override
    public void runOp() throws InterruptedException {
        magnetSensor = RC.h.get(DigitalChannel.class, "sensor_digital");
        magnetSensor.setMode(DigitalChannel.Mode.INPUT);
        // get a reference to our digitalTouch object.
        Armstrong armstrong = new Armstrong();
        // wait for the start button to be pressed.
        waitForStart();
        clearTimer(2);

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {


            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.
            while (armstrong.magnetSensor.getState() && getSeconds(2)<12) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
                armstrong.lifterUp();
                armstrong.collectServoLeftSlow();
                armstrong.collectServoRightSlow();
                RC.t.addData(getSeconds(2));
            }
            armstrong.lifterStop();
            armstrong.unlatch();
            sleep(1000);
            armstrong.forward(0.5);
            sleep(3000);
            armstrong.markDown();
            armstrong.stop();











        }



    }
}

