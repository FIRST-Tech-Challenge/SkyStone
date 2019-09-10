package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Crater Facing", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
@Deprecated
@Disabled
public class CraterFacing extends AutoBase
{
    @Override
    public void runOpMode(){
        //Init's robot
        initLogic();
        while (opModeIsActive()){
            dropDownFromLander();
            robot.intake.setPower(-0.6);
            knockOffMineral(35);
            navigateToDepotThenCrater();
            break;
        }
    }

    //private constructor.
    protected void navigateToDepotThenCrater() {
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.CENTER){
            robot.finalMove(0.6, -48);
        }else {
            robot.finalMove(0.6, -55);
        }

        //Getting to Depot
        robot.finalTurn(65,3250);
        robot.goToWall(0.8,25);
        robot.finalTurn(135);
        robot.moveRobotKillSwitch(0.7,120,-120);
        robot.goToCrater(-0.7);
        telemetry.addData("Status","done");
        telemetry.update();
    }
}
