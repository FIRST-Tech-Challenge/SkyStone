package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.RR2;

@Autonomous(name="ImuInches", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
@Disabled
@Deprecated

public class ImuInches extends AutoBase {
    @Override
    public void runOpMode(){
        //Init's robot
        initLogic();
        while (opModeIsActive()){
            robot.intializeIMU();

        }
    }

    //private constructor.
}



