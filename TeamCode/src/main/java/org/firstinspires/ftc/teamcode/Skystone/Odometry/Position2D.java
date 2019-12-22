package org.firstinspires.ftc.teamcode.Skystone.Odometry;

import android.os.AsyncTask;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class Position2D{
    Robot robot;
    NewThread newThread;
    public Position2D(Robot robot) {
        this.robot = robot;
        Odometry o = new Odometry();
        newThread = new NewThread(robot,o);
    }

    public void startOdometry(){
        newThread.execute();
    }
}
class NewThread extends AsyncTask<Void, Boolean, Boolean> {
    Robot robot;
    Odometry o;
    public NewThread(Robot robot, Odometry o){
        this.robot = robot;
        this.o = o;
    }

    @Override
    protected Boolean doInBackground(Void... params) {
        while(robot.getLinearOpMode().opModeIsActive()) {
            o.circularOdometry(robot);
            robot.setRobotPos(new Point(o.worldX, o.worldY));
            robot.setAnglePos(o.worldAngle);
        }
        return true;
    }

    protected void onPostExecute(Boolean result) {
        if(result) {
            robot.getTelemetry().addLine("DONE");
            robot.getTelemetry().update();
        }
    }

}

