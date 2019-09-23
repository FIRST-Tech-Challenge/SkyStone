//Jun Park
package org.firstinspires.ftc.teamcode.Code;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "ControlMode", group = "Iterative Opmode")
public class ControlMode extends FunctionClass {
    FunctionClass FC = new FunctionClass(); //

    public ControlMode() {
        super();
    }

    public void init() {
        FC.init();
    }

    public void loop() { // This code will executes over and over after the start button is pressed.
        FC.controlMovement();
        FC.reportData();
    }
}
