package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.subsystemutils.Subsystem;

/**
 * assumes that up means extend, down means retract
 */
public class Elevator implements Subsystem {
    private DcMotor lift;
    private Gamepad manipController;

    public Elevator(Gamepad manipController,  DcMotor lift) {
        this.manipController = manipController;
        this.lift = lift;
    }

    @Override
    public void init()
    {
        //does nothing
    }

    @Override
    public void update() {
        double rightStickY = manipController.right_stick_y;
        if(rightStickY > 0) {
            lift.setPower(1);
        }
        else if (rightStickY < 0) {
            lift.setPower(-1);
        }
        else {
            lift.setPower(0);
        }
    }
}
