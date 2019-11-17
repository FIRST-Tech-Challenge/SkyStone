package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.auto.ChassisConfig;
import org.firstinspires.ftc.teamcode.autoOp.SafetyPatrol;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="SafetyPatrol", group="ZZTesting")
public class TylerSafetyPatrol extends SafetyPatrol {
    public TylerSafetyPatrol() {
        super(ChassisConfig.forTileRunner());
    }
}