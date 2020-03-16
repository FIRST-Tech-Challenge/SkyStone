package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous

public class AAAAAAAAAMotorTest extends AutoOpMode {
    private VoltageSensor ExpansionHub3_VoltageSensor;
    public void runOp() throws InterruptedException {
        ExpansionHub3_VoltageSensor =  hardwareMap.voltageSensor.get("Expansion Hub 3");
        Motor motor;
        motor = new Motor("motor");

        telemetry.addData("Status", "initialized");

        motor.resetEncoder();


        waitForStart();


        telemetry.addData("motor ticks before", motor.getPosition());

        motor.setPower(1);
        sleep(2000);
        motor.setPower(0);

        while (opModeIsActive()){
            telemetry.addData("motor ticks after", motor.getPosition());
        }







    }

}
