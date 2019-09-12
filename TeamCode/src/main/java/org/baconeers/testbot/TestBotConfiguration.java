package org.baconeers.testbot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.baconeers.common.RobotConfiguration;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * It is assumed that there is a configuration that is currently activated on the robot controller
 * (run menu / Configure Robot ) with the same name as this class.
 * It is also assumed that the device names in the 'init()' method below are the same as the devices
 * named on the activated configuration on the robot.
 */
public class TestBotConfiguration extends RobotConfiguration {
       // Left motors
    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public TouchSensor touchSensor;


    public ColorSensor sorterColorSensor;
//    public Servo sorterServo;
//    public CRServo blueCRServo;

    /**
     * Assign your class instance variables to the saved device names in the hardware map
     *
     * @param hardwareMap
     * @param telemetry
     */
    @Override
    protected void init(HardwareMap hardwareMap, Telemetry telemetry) {

        setTelemetry(telemetry);

        leftMotor = (DcMotor) getHardwareOn("LeftMotor", hardwareMap.dcMotor);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor = (DcMotor) getHardwareOn("RightMotor", hardwareMap.dcMotor);

        touchSensor = (TouchSensor)getHardwareOn("Touch1", hardwareMap.touchSensor);

//        sorterColorSensor = hardwareMap.colorSensor.get("color");
//        sorterServo = hardwareMap.servo.get("servo");
//
//        blueCRServo = hardwareMap.crservo.get("BackServo");
//        redCRServo = hardwareMap.crservo.get("LeftServo");
    }


    /**
     * Factory method for this class
     *
     * @param hardwareMap
     * @param telemetry
     * @return
     */
    public static TestBotConfiguration newConfig(HardwareMap hardwareMap, Telemetry telemetry) {

        TestBotConfiguration config = new TestBotConfiguration();
        config.init(hardwareMap, telemetry);
        return config;
    }


}
