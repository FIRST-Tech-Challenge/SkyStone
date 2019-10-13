package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;


import java.util.HashMap;
import java.util.Map;

import org.firstinspires.ftc.teamcode.subsystems.RobotMap.IntakeMotor;

public class Intake {
    HashMap<RobotMap.IntakeMotor, DcMotor> motors;

    public Intake(HardwareMap hardwareMap, HashMap<IntakeMotor, String> motorsNames) {
        motors = new HashMap<>();
        for (Map.Entry<IntakeMotor, String> motorName : motorsNames.entrySet()) {
            motors.put(motorName.getKey(), hardwareMap.dcMotor.get(motorName.getValue()));
        }
    }

    public void setMotors(HashMap<IntakeMotor, Double> powers) {
        for (Map.Entry<IntakeMotor, Double> power : powers.entrySet()) {
            motors.get(power.getKey()).setPower(power.getValue());
        }
    }

    public void runIntake(final double power) {
        setMotors(new HashMap<IntakeMotor, Double>() {{
            put(IntakeMotor.LEFT, power);

        }});
    }
}