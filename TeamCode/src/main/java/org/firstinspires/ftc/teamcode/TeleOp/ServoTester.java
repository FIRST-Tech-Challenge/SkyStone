package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name="Servo Tester")
public class ServoTester extends LinearOpMode {
    int index = 0;
    double speed = 0.05;
    ElapsedTime elapsedTime = new ElapsedTime();
    HardwareMap rw;
    boolean a, b;
    @Override
    public void runOpMode() {
        rw = new HardwareMap(hardwareMap);
        Servo[] servos = new Servo[]{
                rw.clawServo1,
                rw.clawServo2,
                rw.transferLock,
                rw.transferHorn,
                rw.clawInit,
                rw.innerTransfer,
                rw.foundationLock,
                rw.intakeInit,
                rw.redAutoClawJoint1,
                rw.redAutoClawJoint2,
                rw.redAutoClawJoint3,
                //rw.parkingServo,
                rw.liftOdometry
        };

        List<String> servoNames = new ArrayList<>();
        List<Double> servoPositions = new ArrayList<>();
        double[] pos = new double[] {TeleopConstants.clawServo1PosClose, TeleopConstants.clawServo2PosClose,
                TeleopConstants.transferLockPosUp, TeleopConstants.transferHornPosReady,
                TeleopConstants.clawInitPosCapstone, TeleopConstants.innerTransferPosTucked,
                TeleopConstants.foundationLockUnlock, TeleopConstants.intakeInitPosReset, TeleopConstants.autoClaw1Extended,
                TeleopConstants.autoClaw2Init, TeleopConstants.autoClaw3Init, TeleopConstants.liftOdometryDown};

        for(int i = 0; i < servos.length; i++) {
            servoPositions.add(pos[i]);
            servoNames.add(hardwareMap.getNamesOf(servos[i]).iterator().next());
        }

        waitForStart();
        elapsedTime.reset();
        double previousTime = 0;
        while (opModeIsActive()) {
            if (!gamepad1.a && a) {
                index++;
            }
            if (!gamepad1.b && b) {
                index--;
            }

            index = ((index % servos.length + servos.length) % servos.length);
            a = gamepad1.a;
            b = gamepad1.b;
            servos[index].setPosition(servoPositions.get(index));
            double deltaTime = (elapsedTime.milliseconds() - previousTime) / 1000;
            previousTime = elapsedTime.milliseconds();

            double newPosition = servoPositions.get(index) + gamepad1.left_stick_y * deltaTime * speed;
            servoPositions.set(index, newPosition);

            telemetry.addData("Number of servos: ", servos.length);
            telemetry.addData("Name of selected servo: ", servoNames.get(index));
            telemetry.addData("Servo position", servoPositions.get(index));
            RobotLogger.dd("Number of Servos", String.valueOf(servos.length));
            RobotLogger.dd("Name of Selected Servo", servoNames.get(index));
            RobotLogger.dd("Servo position", String.valueOf(servoPositions.get(index)));
            telemetry.update();
        }
    }
}