package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Crater Facing 65 Point", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
@Deprecated
@Disabled
public class CraterFacing65Point extends AutoBase
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

    protected void navigateToCrater(){
        //go to crater
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
            craterTurn(7);
            robot.finalMove(0.5,4);
        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
            craterTurn(-7);
            robot.finalMove(0.5,4);
        } else {
            robot.finalMove(0.5,4);
        }
        robot.pivot.setPower(1);
        robot.pivot2.setPower(-1);
        sleep(800);
        robot.pivot2.setPower(0);
        robot.pivot.setPower(0);
        robot.slide.setPower(-1);
        sleep(1300);
        robot.slide.setPower(0);
        robot.intake.setPower(-1);
        sleep(2000);
    }

    protected void craterTurn(double angle){
        robot.finalTurn(angle);
    }
}



