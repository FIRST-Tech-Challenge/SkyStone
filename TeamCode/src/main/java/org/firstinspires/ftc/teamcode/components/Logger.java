package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.utilities.TrcDbgTrace;

/*

SPECIFIER	APPLIES TO	OUTPUT
%a	floating point (except BigDecimal)	Hex output of floating point number
%b	Any type	“true” if non-null, “false” if null
%c	character	Unicode character
%d	integer (incl. byte, short, int, long, bigint)	Decimal Integer
%e	floating point	decimal number in scientific notation
%f	floating point	decimal number
%g	floating point	decimal number, possibly in scientific notation depending on the precision and value.
%h	any type	Hex String of value from hashCode() method.
 %n	none	Platform-specific line separator.
%o	integer (incl. byte, short, int, long, bigint)	Octal number
%s	any type	String value
%t	Date/Time (incl. long, Calendar, Date and TemporalAccessor)	%t is the prefix for Date/Time conversions. More formatting flags are needed after this. See Date/Time conversion below.
%x	integer (incl. byte, short, int, long, bigint)
Hex string.
 */

public class Logger {

    private boolean traceEnabled = false;
    private boolean telemetryEnabled = false;
    private boolean fileOpen = false;
    private String filePrefix = "Logger";
    private TrcDbgTrace tracer;
    private OpMode opMode = null;

    private int DEFAULT_DEBUG_FILTER_THRESHOLD = 50;
    private String DEFAULT_DEBUG_FILTER_FUNCTION = "N/A";
    private int debugFilterCount = 0;
    private int debugFilterThreshold = 0;
    private String debugFilterFunction = DEFAULT_DEBUG_FILTER_FUNCTION;

    /* Constructor */
    public Logger(String aFilePrefix) {
        filePrefix = aFilePrefix;
        open(traceEnabled);
    }

    public Logger(String aFilePrefix, OpMode aOpMode) {
        filePrefix = aFilePrefix;
        open(traceEnabled);
        opMode = aOpMode;
    }

    public Logger(String aFilePrefix, OpMode aOpMode, boolean enableTrace, boolean enableTelemetry) {
        filePrefix =  aFilePrefix;
        opMode = aOpMode;
        open(filePrefix, enableTrace, enableTelemetry);
    }

    public void open(boolean enableTrace) {
        open(filePrefix, enableTrace, telemetryEnabled);
    }

    public void setDebugFilter(String functionName) {
        setDebugFilter(functionName, DEFAULT_DEBUG_FILTER_THRESHOLD);
    }

    public void setDebugFilter (String functionName, int threshold) {
        debugFilterCount = 0;
        debugFilterFunction = functionName;
        debugFilterThreshold = threshold;
    }

    public void incrementDebugFilterCount() {
        debugFilterCount ++;
        if (debugFilterCount > debugFilterThreshold) {
            debugFilterCount = 0;
        }
    }

    public void clearDebugFilter () {
        debugFilterCount = 0;
        debugFilterThreshold = 0;
        debugFilterFunction = DEFAULT_DEBUG_FILTER_FUNCTION;
    }

    public void open(String filePrefix, boolean enableTrace, boolean enableTelemetry) {

        traceEnabled = enableTrace;
        telemetryEnabled = enableTelemetry;

        if (traceEnabled && !fileOpen) {
            tracer = new TrcDbgTrace("LOGGER", traceEnabled, TrcDbgTrace.TraceLevel.HIFREQ, TrcDbgTrace.MsgLevel.VERBOSE);
            tracer.openTraceLog("/sdcard/FIRST/tracelog", filePrefix);
            fileOpen = true;
            if (opMode != null) {
                logInfo("Logger.open","===== [ %s : %s ]", filePrefix, opMode.getClass().getSimpleName());
            } else {
                logInfo("Logger.open","===== [ %s ]", filePrefix);

            }
        }
    }

    private void logToTelemetry(String funcName, String format, Object... args){
        if (opMode != null && telemetryEnabled) {
            opMode.telemetry.addData(funcName, format, args);
        }
    }

    public void logErr(final String funcName, final String format, Object... args) {
        if (traceEnabled) {
            tracer.traceErr(funcName, format, args);
        }
        logToTelemetry(funcName, format, args);
    }

    public void logInfo(final String funcName, final String format, Object... args) {
        if (traceEnabled) {
            tracer.traceInfo(funcName, format, args);
        }
        logToTelemetry(funcName, format, args);
    }

    public void logDebug(final String funcName, final String format, Object... args) {
        if (traceEnabled) {
            if (debugFilterThreshold > 0 && debugFilterFunction == funcName) {

                if (debugFilterCount == 0) {
                    String formatString = ("[").concat(String.valueOf(debugFilterThreshold)).concat("] ").concat(format);
                    tracer.traceVerbose(funcName, formatString, args);
                }

            } else {
                tracer.traceVerbose(funcName, format, args);
            }
        }
     }

    public void close() {
        if (traceEnabled) {
            tracer.closeTraceLog();
        }
    }

}
