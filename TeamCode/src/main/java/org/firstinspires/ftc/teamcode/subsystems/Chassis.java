package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.subsystems.RobotMap.ChassisMotor;

public class Chassis {
    //Vars
    HashMap<ChassisMotor, DcMotor> motors;

    //Constructors
    public Chassis() {
        motors = new HashMap<>();
    }

    public Chassis(HardwareMap hardwareMap, HashMap<ChassisMotor, String> motorsNames) {
        motors = new HashMap<>();
        for (Map.Entry<ChassisMotor, String> motorName : motorsNames.entrySet()) {
            motors.put(motorName.getKey(), hardwareMap.dcMotor.get(motorName.getValue()));
        }
    }

    public HashMap<ChassisMotor, DcMotor> getMotors() {
        return motors;
    }

    //Methods
    public void setPowers(HashMap<ChassisMotor, Double> powers) {
        for (Map.Entry<ChassisMotor, Double> power : powers.entrySet()) {
            motors.get(power.getKey()).setPower(power.getValue());
        }
    }

    public void setTargetPosition(HashMap<ChassisMotor, Integer> positions) {
        for (Map.Entry<ChassisMotor, Integer> position : positions.entrySet()) {
            motors.get(position.getKey()).setPower(position.getValue());
        }
    }

    public void reverseMotors(ChassisMotor[] reverseMotors) {
        for (ChassisMotor reverseMotor : reverseMotors) {
            motors.get(reverseMotor).setDirection(DcMotor.Direction.REVERSE);
        }

    }

    public boolean runChassis(final double angle, final double turn, final double power) {
        final double frontLeftPower = power * Math.cos(angle) + turn;
        final double frontRightPower = power * Math.sin(angle) - turn;
        final double backLeftPower = power * Math.sin(angle) + turn;
        final double backRightPower = power * Math.cos(angle) - turn;
        setPowers(new HashMap<ChassisMotor, Double>() {{
            put(RobotMap.ChassisMotor.FRONT_LEFT, frontLeftPower);
            put(RobotMap.ChassisMotor.FRONT_RIGHT, frontRightPower);
            put(RobotMap.ChassisMotor.BACK_LEFT, backLeftPower);
            put(RobotMap.ChassisMotor.BACK_RIGHT, backRightPower);
        }});
        return true;
    }

    public void reset() {
        for (Map.Entry<ChassisMotor, DcMotor> motor : motors.entrySet()) {
            motor.getValue().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (Map.Entry<ChassisMotor, DcMotor> motor : motors.entrySet()) {
            motor.getValue().setMode(runMode);
        }
    }


}
