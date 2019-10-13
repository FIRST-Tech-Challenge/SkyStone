package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Central extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();

    public Crane rob;

    public Crane.movements[] allMovements = {Crane.movements.forward, Crane.movements.backward, Crane.movements.right, Crane.movements.left, Crane.movements.tr, Crane.movements.bl, Crane.movements.tl, Crane.movements.br,Crane.movements.cw, Crane.movements.ccw};

    public void setRob(Crane rob) {
        this.rob = rob;
    }

    public void setup(ElapsedTime rtime, Crane.setupType... setup) throws InterruptedException {
        this.setRob(new Crane(hardwareMap, runtime, this, setup));
        setRuntime(rtime);
        /*if (rob.vuforiaMode){
            //    rob.vuforia.targetsRoverRuckus.activate();
        }
        if (rob.tensorflowMode){
            rob.vuforia.tfod.activate();
        }*/
        this.waitForStart();
        this.runtime.reset();


    }

    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }


}
