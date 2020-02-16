package org.firstinspires.ftc.teamcode.Skystone.Odometry;

import android.os.AsyncTask;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class Position2D {
    Robot robot;
    NewThread newThread;
    public Odometry o;

    public Position2D(Robot robot) {
        this.robot = robot;
        o = new Odometry();
        newThread = new NewThread(robot, o);
    }

    public void startOdometry() {
        newThread.execute();
    }
}

class NewThread extends AsyncTask<Void, Boolean, Boolean> {
    Robot robot;
    Odometry o;
    Point newPoint;
    static int count = 0;

    public NewThread(Robot robot, Odometry o) {
        this.robot = robot;
        this.o = o;
        newPoint = new Point();
    }

    @Override
    protected Boolean doInBackground(Void... params) {
        while (robot.getLinearOpMode().opModeIsActive()) {
            o.runOdometry(robot);
            newPoint.x = o.worldX;
            newPoint.y = o.worldY;
            robot.setRobotPos(newPoint);
            robot.setAnglePos(o.worldAngle);
            if (robot.isDebug()) {
                robot.addOdometryPoints(newPoint.x, newPoint.y);
            }
//            if ((count%5) == 0){
//                robot.addOdometryPoints(o.worldX, o.worldY);
//            }
//            count++;
        }
        return true;
    }

    protected void onPostExecute(Boolean result) {
        if (result) {
            robot.getTelemetry().addLine("DONE");
            robot.getTelemetry().update();
        }
    }

}

