package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.All.HardwareMap;

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
                rw.liftOdometer
        };//hardwareMap.getAll(Servo.class);

        List<String> servoNames = new ArrayList<>();
        List<Double> servoPositions = new ArrayList<>();
        for (Servo s : servos) {
            servoPositions.add(0.5);
            servoNames.add(hardwareMap.getNamesOf(s).iterator().next());
        }


        // specifically for this year
        /*RobotHardware rw = new RobotHardware(hardwareMap);
        servos.add(rw.firstJointVirtualServo);
        servos.add(rw.secondJointVirtualServo);
        servoNames.add("First Joint Virtual Servo");
        servoNames.add("Second Joint Virtual Servo");
        servoPositions.add(0d);
        servoPositions.add(0d);*/


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
            telemetry.update();
        }
    }
}