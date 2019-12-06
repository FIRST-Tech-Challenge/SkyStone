package org.firstinspires.ftc.opmodes.testcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumHardwareMap;
import org.firstinspires.ftc.robotlib.state.Button;

@TeleOp(name="Servo Testing", group="Linear Opmode")
public class ServoTesting extends LinearOpMode
{
    private MecanumHardwareMap robotHardware;
    private ElapsedTime elapsedTime;
    private boolean rbUp = true;
    private boolean lbUp = true;
    private Button rightTrigger = new Button();
    private Button yButton = new Button();
    private int index = 0;

    @Override
    public void runOpMode()
    {
        robotHardware = new MecanumHardwareMap(this.hardwareMap);
        elapsedTime = new ElapsedTime();

        double[] servoPositions = new double[]{0.07, 0.15, 0.73, 0.78, 0.89, 1.0};
        String[] positionNames = new String[]{"Cradle", "Carry", "Hover2", "Deposit2/Hover1", "Deposit1", "Floor"};

        while (!isStopRequested()) {
            if (gamepad1.right_bumper) {
                if (rbUp) {
                    rbUp = false;
                    if (index + 1 < servoPositions.length) {
                        index++;
                        robotHardware.deliveryLeft.setPosition(servoPositions[index]);
                        robotHardware.deliveryRight.setPosition(servoPositions[index] + 0.02);
                    }
                }
            } else rbUp = true;

            if (gamepad1.left_bumper) {
                if (lbUp) {
                    lbUp = false;
                    if (index - 1 >= 0) {
                        index--;
                        robotHardware.deliveryLeft.setPosition(servoPositions[index]);
                        robotHardware.deliveryRight.setPosition(servoPositions[index] + 0.02);
                    }
                }
            } else lbUp = true;

            if (rightTrigger.isToggled()) {
                robotHardware.intakeMotorManager.setMotorsVelocity(1.0);
            }
            if (rightTrigger.isReleased()) {
                robotHardware.intakeMotorManager.setMotorsVelocity(0.0);
            }

            if (yButton.isReleased()) {
                if (robotHardware.blockGrabber.getPosition() == 0.0) robotHardware.blockGrabber.setPosition(1.0);
                else robotHardware.blockGrabber.setPosition(0.0);
            }

            yButton.input(gamepad1.y);
            rightTrigger.input(gamepad1.right_trigger > 0);

            telemetry.addData("Info", index + " " + positionNames[index] + " " + servoPositions[index]);
            telemetry.update();

            /*for (double i = 0.0; i < 1.0; i += 0.1) {
                robotHardware.deliveryServoManager.setPosition(i);
                telemetry.addData("DELIVERY SERVO POSITION", robotHardware.deliveryServoManager.getServoState().getPosition());
                double initialTime = elapsedTime.milliseconds();
                while (elapsedTime.milliseconds() < initialTime + (1000)) {}
            }*/
            /*robotHardware.deliveryServoManager.addServoPositions(this.telemetry);
            telemetry.update();*/
        }
        /*for (double i = 0.0; i < 1.0; i += 0.1) {
            robotHardware.deliveryServoManager.setPosition(i);
            telemetry.addData("DELIVERY SERVO POSITION", robotHardware.deliveryServoManager.getServoState().getPosition());
            double initialTime = elapsedTime.milliseconds();
            while (elapsedTime.milliseconds() < initialTime + (1000)) {}
        }*/
        /*robotHardware.deliveryServoManager.setPosition(0.5);
        robotHardware.blockGrabber.setPosition(0.5);

        telemetry.addData("DELIVERY SERVO POSITION", robotHardware.deliveryServoManager.getServoState().toString());
        robotHardware.deliveryServoManager.addServoPositions(this.telemetry);
        telemetry.addData("BLOCK GRABBER POSITION", robotHardware.blockGrabber.getPosition());
        telemetry.update();*/
    }
}