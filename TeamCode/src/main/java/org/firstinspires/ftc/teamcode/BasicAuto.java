package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BasicAuto", group = "Basic")
public class BasicAuto extends LinearOpMode {

    TypexChart chart = new TypexChart();
    ElapsedTime time = new ElapsedTime();

    public void Drive(double time) {
        
    }

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while(opModeIsActive()) {

        }
    }
}
