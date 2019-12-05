package org.firstinspires.ftc.opmodes.testcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.MecanumHardwareMap;

@TeleOp(name="Servo Testing", group="Linear Opmode")
public class ServoTesting extends LinearOpMode
{
    private MecanumHardwareMap robotHardware;
    private ElapsedTime elapsedTime;
    private boolean rbUp = true;
    private boolean lbUp = true;
    private int index = 0;

    @Override
    public void runOpMode()
    {
        robotHardware = new MecanumHardwareMap(this.hardwareMap);
        elapsedTime = new ElapsedTime();

        double[] servoPositions = new double[]{0.3, 0.4, 0.5, 0.6, 0.7};

        while (!isStopRequested()) {
            if (gamepad1.right_bumper) {
                if (rbUp) {
                    rbUp = false;
                    if (index + 1 < servoPositions.length) {
                        index++;
                        robotHardware.deliveryServoManager.setPosition(servoPositions[index]);
                    }
                }
            } else rbUp = true;

            if (gamepad1.left_bumper) {
                if (lbUp) {
                    lbUp = false;
                    if (index > 0) {
                        index--;
                        robotHardware.deliveryServoManager.setPosition(servoPositions[index]);
                    }
                }
            } else lbUp = true;

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