package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Depot Facing", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
@Deprecated
@Disabled
public class DepotFacing extends AutoBase
{
    double distanceToDepot = 77;
    boolean center = false;
    @Override
    public void runOpMode(){
        initLogic();
        while (opModeIsActive()){
            dropDownFromLander();
            knockOffMineral(35);
            navigateToDepotThenCrater(distanceToDepot);
            break;
        }
    }

    protected void navigateToDepotThenCrater(double distance) {
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
            robot.finalMove(0.7, -53);
            robot.finalTurn(75);
            //Getting to Depot
        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
            distanceToDepot = 65;
            robot.finalTurn(65,5000);
        } else {
            center = true;
            distance = 0;
            //Getting to Depot
            robot.finalMove(0.7, 39);
            robot.releaseTeamMarker();
            robot.finalMove(0.7, -87);
            robot.finalTurn(70);
        }

        robot.goToWall(0.7, 25);

        if(!center) {
            robot.finalTurn(-65);
            robot.finalMove(0.7,distance);
            robot.releaseTeamMarker();
            robot.goToCrater(-0.5);
        }
        if (center) {
            robot.finalTurn(132);
            robot.finalMove(0.7, 30);
            robot.pivot.setPower(1);
            robot.pivot2.setPower(-1);
            sleep(500);
            robot.pivot2.setPower(0);
            robot.pivot.setPower(0);
            robot.slide.setPower(-1);
            sleep(1000);
            robot.slide.setPower(0);
            robot.intake.setPower(-1);
            sleep(2000);
        }

        telemetry.addData("Status","done");
        telemetry.update();
    }
}