package org.firstinspires.ftc.teamcode.Skystone.Test;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Skystone.Robot;

import java.util.concurrent.CancellationException;

@Autonomous(name="TestFileWriting", group ="LinearOpmode")
public class TestFileWriting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        try {
            Robot.writeToFile("hi" + SystemClock.elapsedRealtime() + ".txt", "hi");
            sleep(3000);
        } catch (Exception e){
            Robot.writeToFile("bye" + SystemClock.elapsedRealtime() + ".txt", "bye");
        }
    }

}
