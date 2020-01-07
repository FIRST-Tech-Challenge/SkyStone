package org.firstinspires.ftc.teamcode.SubAssembly.Lift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class LiftControl {/* Constants */
    final double LIFT_SPEED = 1.0;

    /* Declare private class object */
    private LinearOpMode opmode = null;     /* local copy of opmode class */

    private DcMotor LifterRightM;
    private DcMotor LifterLeftM;
    private TouchSensor LifterButtonT;
    private TouchSensor LifterButtonB;

    /* Declare public class object */


    /* Subassembly constructor */
    public LiftControl() {
    }

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;

        opMode.telemetry.addLine("Lift Control" + " initialize");
        opMode.telemetry.update();

        /* Set local copies from opmode class */
        opmode = opMode;
        hwMap = opMode.hardwareMap;

        /* Map hardware devices */
        LifterRightM = hwMap.dcMotor.get("LifterRightM");
        LifterLeftM = hwMap.dcMotor.get("LifterLeftM");
        LifterRightM.setDirection(DcMotor.Direction.FORWARD);
        LifterLeftM.setDirection(DcMotor.Direction.FORWARD);
        LifterRightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LifterLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LifterRightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LifterLeftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LifterRightM.setPower(0);
        LifterLeftM.setPower(0);

        LifterButtonB = hwMap.touchSensor.get("LifterButtonB");
        LifterButtonT = hwMap.touchSensor.get("LifterButtonT");

        //LifterLeftM.setDirection(DcMotor.Direction.REVERSE);
    }

    public void MoveUp() {
        LifterLeftM.setPower(LIFT_SPEED);
        LifterRightM.setPower(LIFT_SPEED);
    }

    /* time NOT implemented
    public void MoveUpTime (double time){
        LifterLeftM.setPower(LIFT_SPEED);
        LifterRightM.setPower(LIFT_SPEED);
    }
    */

    public void MoveDown() {
        LifterLeftM.setPower(-LIFT_SPEED);
        LifterRightM.setPower(-LIFT_SPEED);
    }

    public void Stop() {
        LifterLeftM.setPower(0);
        LifterRightM.setPower(0);
    }

    public boolean isLimitTop() {
        // !!! current hardware configuration has isPressed returning
        //     TRUE when the limit switch is NOT pressed
        return !LifterButtonT.isPressed();
    }

    public boolean isLimitBottom() {
        // !!! current hardware configuration has isPressed returning
        //     TRUE when the limit switch is NOT pressed
        return !LifterButtonB.isPressed();
    }
}
