package org.firstinspires.ftc.teamcode.PreseasonTest.MecanumLocalizers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;

import de.siegmar.fastcsv.writer.CsvAppender;
import de.siegmar.fastcsv.writer.CsvWriter;

public class BulkReadManager {

    private ExpansionHubEx expansionHub;

    // high priority callbacks are processed first (meant for sensors)
    // low priority are then processed (things that are dependent on sensor values)
    private ArrayList<BulkReadConsumer> highPriorityCallbacks;
    private ArrayList<BulkReadConsumer> lowPriorityCallbacks;

    private long lastPoll;
    private CsvWriter csvWriter;
    private File logfile;

    private RevBulkData revBulkData;

    public void init(HardwareMap hwMap) {
        // TODO add name of expansion hub
        expansionHub = hwMap.get(ExpansionHubEx.class, "");
        highPriorityCallbacks = new ArrayList<>();
        lowPriorityCallbacks = new ArrayList<>();

        logfile = new File(AppUtil.LOG_FOLDER + "poll.txt");
        csvWriter = new CsvWriter();

    }

    public void registerHighPriority(BulkReadConsumer callback) {
        highPriorityCallbacks.add(callback);
    }

    public void registerLowPriority(BulkReadConsumer callback) {
        lowPriorityCallbacks.add(callback);
    }

    public void poll() {
        // Measure polling speed
        double elapsedTime = (System.nanoTime() - lastPoll) / Math.pow(10, 9);
        double pollrate = 1 / elapsedTime;

        try (CsvAppender csvAppender = csvWriter.append(logfile, StandardCharsets.UTF_8)){
            csvAppender.appendLine(String.valueOf(elapsedTime), String.valueOf(pollrate));
        }
        catch (IOException e) {
            RobotLog.e("Failed to write to log file");
        }

        revBulkData = expansionHub.getBulkInputData();

        for (BulkReadConsumer callback : highPriorityCallbacks) {
            callback.bulkReadUpdate(revBulkData);
        }

        for (BulkReadConsumer callback : lowPriorityCallbacks) {
            callback.bulkReadUpdate(revBulkData);
        }

        lastPoll = System.nanoTime();
    }

    public RevBulkData getRevBulkData() {
        return revBulkData;
    }
}
