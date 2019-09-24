//Jun Park
package org.firstinspires.ftc.teamcode.Code;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ControlMode", group = "Iterative Opmode")
public class ControlMode extends FunctionClass {
    FunctionClass FC = new FunctionClass(); // Use the functions from the FunctionClass

    public ControlMode() {
        super();
    }

    public void init() {
        FC.init(); // Set up the motors
    }

    public void loop() {
        FC.controlMovement(); // Move the vehicle
        FC.reportData(); // Report the detailed data to the controller phone
    }
}
