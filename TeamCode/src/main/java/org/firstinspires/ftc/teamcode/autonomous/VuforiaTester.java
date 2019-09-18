package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class VuforiaTester extends OpMode {

    VuforiaSensor vSensor = new VuforiaSensor(telemetry);

    public VuforiaTester() {
        super();
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {
        vSensor.activate();
    }

    @Override
    public void loop() {
        vSensor.loop();
    }

}