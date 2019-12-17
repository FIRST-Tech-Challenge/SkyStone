package org.firstinspires.ftc.robotlib.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.controller.PIDController;
import org.firstinspires.ftc.robotlib.drivetrain.Odometrical;
import org.firstinspires.ftc.robotlib.state.Button;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;
import org.firstinspires.ftc.robotlib.state.ToggleInt;

public class PIDTuner
{
    private static final double ROTATION_POWER = 0.5;
    private Odometrical drivetrain;
    private PIDController controller;
    private Gamepad gamepad;
    private Telemetry telemetry;
    private double stepP = 1, stepI = 0.01, stepD = 0.01;
    private Button upButton = new Button();
    private Button downButton = new Button();
    private Button leftButton = new Button();
    private Button rightButton = new Button();
    private Button yButton = new Button();
    private ToggleBoolean aButton = new ToggleBoolean();
    private ToggleInt selected = new ToggleInt(3);

    public PIDTuner(Odometrical drivetrain, PIDController controller, Gamepad gamepad, Telemetry telemetry)
    {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        drivetrain.setTargetHeading(0);
    }

    public void update()
    {
        upButton.input(gamepad.dpad_up);
        downButton.input(gamepad.dpad_down);
        leftButton.input(gamepad.dpad_left);
        rightButton.input(gamepad.dpad_right);
        yButton.input(gamepad.y);
        selected.input(gamepad.x);
        aButton.input(gamepad.a);

        if (upButton.isPressed())
        {
            switch (selected.output())
            {
                case 0:
                    controller.setKP(controller.getKP()+stepP);

                case 1:
                    controller.setKI(controller.getKI()+stepI);

                case 2:
                    controller.setKD(controller.getKD()+stepD);
            }
        }
        if (downButton.isPressed())
        {
            switch (selected.output())
            {
                case 0:
                    controller.setKP(controller.getKP()+stepP);

                case 1:
                    controller.setKI(controller.getKI()+stepI);

                case 2:
                    controller.setKD(controller.getKD()+stepD);
            }
        }

        if (yButton.isPressed())
        {
            controller.resetIntegration();
        }

        if (aButton.output())
        {
            telemetry.addData("Status", "Paused");
            drivetrain.setRotation(0);
        }
        else
        {
            telemetry.addData("Status", "Running");
            if (gamepad.left_bumper && gamepad.right_bumper)
            {
                drivetrain.setRotation(0);
            }
            else if (gamepad.left_bumper)
            {
                drivetrain.setRotation(ROTATION_POWER);
            }
            else if (gamepad.right_bumper)
            {
                drivetrain.setRotation(-ROTATION_POWER);
            }
            else
            {
                drivetrain.updateHeading();
            }
        }

        telemetry.addData("Controls" , "X to select, Y to reset, A to pause");

        String[] KCaptions = new String[] {"KP", "KI", "KD"};
        String[] stepCaptions = new String[] {"stepP", "stepI", "stepD"};

        KCaptions[selected.output()] = "->"+KCaptions[selected.output()];
        stepCaptions[selected.output()] = "->"+stepCaptions[selected.output()];

        telemetry.addData(KCaptions[0], controller.getKP());
        telemetry.addData(KCaptions[1], controller.getKI());
        telemetry.addData(KCaptions[2], controller.getKD());
        telemetry.addData(stepCaptions[0], stepP);
        telemetry.addData(stepCaptions[1], stepI);
        telemetry.addData(stepCaptions[2], stepD);
        telemetry.addData("Heading", drivetrain.getCurrentHeading()*1000);
        telemetry.addData("Error", controller.getError()*1000);
        telemetry.addData("Integral", controller.getIntegral());
        telemetry.addData("Derivative", controller.getDerivative());
        telemetry.update();
    }
}
