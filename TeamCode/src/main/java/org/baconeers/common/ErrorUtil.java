package org.baconeers.common;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Util to prevent unhandled exceptions from crashing the app
 */
public class ErrorUtil {

    public static void handleCatchAllException(Throwable e, Telemetry telemetry) throws InterruptedException {
        telemetry.log().add("Opmode Exception:" + e.getMessage());
        String stckTrace = stackTraceAsString(e);
        telemetry.log().add("Opmode Stacktrace: " + stckTrace.substring(0, 200));

        if (e instanceof InterruptedException) {
            throw (InterruptedException) e;
        }

    }

    private static String stackTraceAsString(Throwable e) {

        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        e.printStackTrace(pw);
        return sw.toString();

    }

}
