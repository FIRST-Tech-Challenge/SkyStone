package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Depot Facing 65 Point", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
@Deprecated
@Disabled
public class DepotFacing65Point extends AutoBase
{

    @Override
    public void runOpMode(){
        //Init's robot
        initLogic();
        while (opModeIsActive()){
            dropDownFromLander();
            knockOffMineral(35);
            navigateToCrater();
            break;
        }
    }

    private void navigateToCrater() {
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.CENTER){
            robot.finalMove(0.5, 35);
            robot.releaseTeamMarker();
        }

        robot.finalMove(0.5, -60);

        telemetry.addData("Status","done");
        telemetry.update();
    }
}
