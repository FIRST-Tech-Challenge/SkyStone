package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.List;
import java.util.Map;

@Autonomous(name = "Hardware Tester", group = "none")
public class HardwareTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        List<HardwareMap.DeviceMapping<? extends HardwareDevice>> deviceMappings = hardwareMap.allDeviceMappings;
        for (HardwareMap.DeviceMapping deviceMapping : deviceMappings) {
            for (Object objEntry : deviceMapping.entrySet()) {
                Map.Entry<String, HardwareDevice> entry = (Map.Entry<String, HardwareDevice>) objEntry;
                telemetry.addData("Device Name", entry.getKey());
                HardwareDevice device = entry.getValue();
                telemetry.addData("Device Type", device.getDeviceName());
                telemetry.addData("Device Class", device.getClass());
                telemetry.update();
                sleep(2000);
                if (device instanceof DcMotorSimple) {
                    DcMotorSimple motor = (DcMotorSimple) device;
                    telemetry.addLine("Turning Motor Forward...");
                    telemetry.update();
                    motor.setPower(1);
                    sleep(500);
                    telemetry.addLine("Pausing...");
                    telemetry.update();
                    motor.setPower(1);
                    sleep(250);
                    telemetry.addLine("Turning Motor Backward...");
                    telemetry.update();
                    motor.setPower(-1);
                    sleep(500);
                    motor.setPower(0);
                } else if (device instanceof Servo) {
                    Servo servo = (Servo) device;
                    telemetry.addLine("Turning Servo Forward...");
                    telemetry.update();
                    servo.setPosition(1);
                    sleep(500);
                    telemetry.addLine("Turning Servo Backward...");
                    telemetry.update();
                    servo.setPosition(0);
                    sleep(500);
                } else if (device instanceof GyroSensor) {
                    GyroSensor gyro = (GyroSensor) device;
                    telemetry.addData("Integrated Z heading", gyro.getHeading());
                    telemetry.update();
                    sleep(1000);
                } else if (device instanceof ColorSensor) {
                    ColorSensor color = (ColorSensor) device;
                    telemetry.addData("Red", color.red());
                    telemetry.addData("Green", color.green());
                    telemetry.addData("Blue", color.blue());
                    telemetry.update();
                    sleep(1000);
                }
                else {
                    telemetry.addLine("No tests performed, unregistered device type.");
                    telemetry.update();
                    sleep(1000);
                }
            }
        }
    }
}
