package org.firstinspires.ftc.teamcode.Skystone.Odometry;

import android.os.AsyncTask;
import android.os.SystemClock;
import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    public NewThread(Robot robot, Odometry o) {
        this.robot = robot;
        this.o = o;
        newPoint = new Point();
    }

    @Override
    protected Boolean doInBackground(Void... params) {
        boolean isStopIntake = false;
        long stopIntakeTime = 0;
        while (robot.getLinearOpMode().opModeIsActive()) {
            o.runOdometry(robot);
            newPoint.x = o.worldX;
            newPoint.y = o.worldY;
            robot.setRobotPos(newPoint);
            robot.setAnglePos(o.worldAngle);

            if(robot.getIntakeLeft().getPower() != 0 && !isStopIntake) {
//                long startTime = SystemClock.elapsedRealtime();
                if (robot.getIntakeStoneDistance().getDistance(DistanceUnit.CM) < 20) {
                    isStopIntake = true;
                    stopIntakeTime = SystemClock.elapsedRealtime();
                }
//                Log.d("Distance", Long.toString(SystemClock.elapsedRealtime() - startTime));
            }

            if(isStopIntake && SystemClock.elapsedRealtime()- stopIntakeTime >= 250){
                if(robot.getIntakeStoneDistance().getDistance(DistanceUnit.CM) < 20){
                    robot.intake(false);
                    isStopIntake = false;
                }else{
                    robot.getIntakeLeft().setPower(0);
                    robot.getIntakeRight().setPower(0);
                    isStopIntake = false;
                }
            }

//            if (robot.isDebug()) {
////                robot.addOdometryPoints(newPoint.x, newPoint.y);
//            }
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

