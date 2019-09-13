
/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.components.utilities;


import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;


/**
 * This class implements the Debug Tracer.
 */
public class TrcDbgTrace
{
    /**
     * This enum specifies the different debug tracing levels. They are used in the traceEnter and traceExit methods.
     */
    public enum TraceLevel
    {
        QUIET(0),
        INIT(1),
        API(2),
        CALLBK(3),
        EVENT(4),
        FUNC(5),
        TASK(6),
        UTIL(7),
        HIFREQ(8);

        private int value;

        TraceLevel(int value)
        {
            this.value = value;
        }   //TraceLevel

        public int getValue()
        {
            return this.value;
        }   //getValue

    }   //enum TraceLevel

    /**
     * This enum specifies the different debug message levels. They are used in the traceMsg methods.
     */
    public enum MsgLevel
    {
        FATAL(1),
        ERR(2),
        WARN(3),
        INFO(4),
        VERBOSE(5);

        private int value;

        MsgLevel(int value)
        {
            this.value = value;
        }   //MsgLevel

        public int getValue()
        {
            return this.value;
        }   //getValue

    }   //enum MsgLevel

    private static int indentLevel = 0;

    private String instanceName;
    private boolean traceEnabled;
    private TraceLevel traceLevel;
    private MsgLevel msgLevel;
    private double nextTraceTime;
    private PrintStream traceLog = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param traceEnabled specifies true to enable debug tracing, false to disable.
     * @param traceLevel specifies the trace level.
     * @param msgLevel specifies the message level.
     */
    public TrcDbgTrace(final String instanceName, boolean traceEnabled, TraceLevel traceLevel, MsgLevel msgLevel)
    {
        this.instanceName = instanceName;
        setDbgTraceConfig(traceEnabled, traceLevel, msgLevel);
        this.nextTraceTime = getCurrentTime();
    }   //TrcDbgTrace

    /**
     * This method opens a log file for writing all the trace messages to it.
     *
     * @param traceLogName specifies the trace log file name.
     * @return true if log file is successfully opened, false if it failed.
     */
    public boolean openTraceLog(final String traceLogName)
    {
        boolean success = true;

        try
        {
            traceLog = new PrintStream(new File(traceLogName));
        }
        catch (FileNotFoundException e)
        {
            traceLog = null;
            success = false;
        }

        return success;
    }   //openTraceLog

    /**
     * This method opens a log file for writing all the trace messages to it. The log file is written to the specified
     * folder. The file name will be formed by concatenating the specified file prefix and a date-time stamp.
     *
     * @param folderPath specifies the folder path.
     * @param filePrefix specifies the file name prefix.
     * @return true if log file is successfully opened, false if it failed.
     */
    public boolean openTraceLog(final String folderPath, final String filePrefix)
    {
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd@HH-mm-ss", Locale.US);
        String logFilePath = folderPath + "/" + filePrefix + "_" + dateFormat.format(new Date()) + ".log";
        File folder = new File(folderPath);
        folder.mkdir();

        return openTraceLog(logFilePath);
    }   //openTraceLog

    /**
     * This method closes the trace log file.
     */
    public void closeTraceLog()
    {
        if (traceLog != null)
        {
            traceLog.close();
            traceLog = null;
        }
    }   //closeTraceLog

    /**
     * This method sets the trace level, message level of the debug tracer. It can also enables/disables function
     * tracing.
     *
     * @param traceEnabled specifies true to enable function tracing, false to disable.
     * @param traceLevel specifies the trace level.
     * @param msgLevel specifies the message level.
     */
    public void setDbgTraceConfig(boolean traceEnabled, TraceLevel traceLevel, MsgLevel msgLevel)
    {
        this.traceEnabled = traceEnabled;
        this.traceLevel = traceLevel;
        this.msgLevel = msgLevel;
    }   //setDbgTraceConfig

    /**
     * This method is typically called at the beginning of a method to trace the entry parameters of the method.
     *
     * @param funcName specifies the calling method name.
     * @param funcLevel specifies the trace level.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceEnter(final String funcName, final TraceLevel funcLevel, final String format, Object... args)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            HalDbgLog.traceMsg(tracePrefix(funcName, true, false) + String.format(format, args) + ")\n");
        }
    }   //traceEnter

    /**
     * This method is typically called at the beginning of a method.
     *
     * @param funcName specifies the calling method name.
     * @param funcLevel specifies the trace level.
     */
    public void traceEnter(final String funcName, final TraceLevel funcLevel)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            HalDbgLog.traceMsg(tracePrefix(funcName, true, true));
        }
    }   //traceEnter

    /**
     * This method is typically called at the end of a method to trace the return value of the method.
     *
     * @param funcName specifies the calling method name.
     * @param funcLevel specifies the trace level.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceExit(final String funcName, final TraceLevel funcLevel, final String format, Object... args)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            HalDbgLog.traceMsg(tracePrefix(funcName, false, false) + String.format(format, args) + "\n");
        }
    }   //traceExitMsg

    /**
     * This method is typically called at the end of a method.
     * @param funcName specifies the calling method name.
     * @param funcLevel specifies the trace level.
     */
    public void traceExit(final String funcName, final TraceLevel funcLevel)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            HalDbgLog.traceMsg(tracePrefix(funcName, false, true));
        }
    }   //traceExit

    /**
     * This method is called to print a fatal message.
     *
     * @param funcName specifies the calling method name.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceFatal(final String funcName, final String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.FATAL, 0.0, format, args);
    }   //traceFatal

    /**
     * This method is called to print an error message.
     *
     * @param funcName specifies the calling method name.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceErr(final String funcName, final String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.ERR, 0.0, format, args);
    }   //traceErr

    /**
     * This method is called to print a warning message.
     *
     * @param funcName specifies the calling method name.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceWarn(final String funcName, final String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.WARN, 0.0, format, args);
    }   //traceWarn

    /**
     * This method is called to print an information message.
     *
     * @param funcName specifies the calling method name.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceInfo(final String funcName, final String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.INFO, 0.0, format, args);
    }   //traceInfo

    /**
     * This method is called to print a verbose message.
     *
     * @param funcName specifies the calling method name.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceVerbose(final String funcName, final String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.VERBOSE, 0.0, format, args);
    }   //traceVerbose

    /**
     * This method is called to print a message only if the given time interval has been passed since the last
     * periodic message. This is useful to print out periodic status without overwhelming the debug console.
     *
     * @param funcName specifies the calling method name.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void tracePeriodic(final String funcName, double traceInterval, final String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.INFO, traceInterval, format, args);
    }   //tracePeriodic

    /**
     * This method prints a debug message to the debug console.
     *
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void tracePrintf(String format, Object... args)
    {
        HalDbgLog.traceMsg(String.format(format, args));
    }   //tracePrintf

    /**
     * This method is the common worker for all the trace message methods.
     *
     * @param funcName specifies the calling method name.
     * @param level specifies the message level.
     * @param traceInterval specifies the tracing interval. If not periodic, this must be set to zero.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    private void traceMsg(
            final String funcName, MsgLevel level, double traceInterval, final String format, Object... args)
    {
        if (level.getValue() <= msgLevel.getValue())
        {
            double currTime = getCurrentTime();
            if (currTime >= nextTraceTime)
            {
                nextTraceTime = currTime + traceInterval;
                String msg = msgPrefix(funcName, level) + String.format(format, args) + "\n";
                HalDbgLog.msg(level, msg);
                if (traceLog != null)
                {
                    traceLog.print(msg);
                    traceLog.flush();
                }
            }
        }
    }   //traceMsg

    /**
     * This method returns a trace prefix string. The trace prefix includes the indentation, the instance name and
     * calling method name.
     *
     * @param funcName specifies the calling method name.
     * @param enter specifies true if it is a traceEnter call, false if it is a traceExit call.
     * @param newline specifies true if it should print a newline, false otherwise.
     * @return trace prefix string.
     */
    private String tracePrefix(final String funcName, boolean enter, boolean newline)
    {
        String prefix = "";

        if (enter)
        {
            indentLevel++;
        }

        for (int i = 0; i < indentLevel; i++)
        {
            prefix += "| ";
        }

        prefix += instanceName + "." + funcName;

        if (enter)
        {
            prefix += newline? "()\n": "(";
        }
        else
        {
            prefix += newline? "!\n": "";
            indentLevel--;
        }

        return prefix;
    }   //tracePrefix

    /**
     * This method returns a message prefix string.
     *
     * @param funcName specifies the calling method name.
     * @param level specifies the message level.
     * @return message prefix string.
     */
    private String msgPrefix(final String funcName, MsgLevel level)
    {
        String prefix = instanceName + "." + funcName;

        switch (level)
        {
            case FATAL:
                prefix += "_Fatal: ";
                break;

            case ERR:
                prefix += "_Err: ";
                break;

            case WARN:
                prefix += "_Warn: ";
                break;

            case INFO:
                prefix += "_Info: ";
                break;

            case VERBOSE:
                prefix += "_Verbose: ";
                break;

            default:
                prefix += "_Unk: ";
                break;
        }

        return prefix;
    }   //msgPrefix

    /**
     * This method returns the current time in seconds with nano-second precision.
     *
     * @return current time in seconds.
     */
    public static double getCurrentTime()
    {
        return System.nanoTime()/1000000000.0;
    }   //getCurrentTime

}   //class TrcDbgTrace
