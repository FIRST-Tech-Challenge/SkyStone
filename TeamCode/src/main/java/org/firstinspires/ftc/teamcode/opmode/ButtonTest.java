package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Button Test")
@Disabled
public class ButtonTest extends LinearOpMode {

    private Button xButton;
    private Button yButton;

    private boolean ifActivated = false;

    public void runOpMode(){
        xButton = new Button(gamepad1, Button.ListenButton.X);
        yButton = new Button(gamepad1, Button.ListenButton.Y);
        waitForStart();

        while(!isStopRequested()){

            telemetry.addData("X: ",xButton.shouldExecuteAction());
            telemetry.addData("Y: ", yButton.shouldExecuteAction());

            if(xButton.shouldExecuteAction() || yButton.shouldExecuteAction()){
                ifActivated = true;
            }

            telemetry.addData("If Any has Activated: ", ifActivated);

            xButton.update();
            yButton.update();

            telemetry.update();

        }

    }

}
