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

import android.util.Log;


/**
 * This class implements the platform dependent debug logging.
 */
public class HalDbgLog
{
    private static final String TAG = "TrcDbg";

    /**
     * This method is called to print a message with the specified message level to the debug console.
     *
     * @param level specifies the message level.
     * @param msg specifies the message.
     */
    public static void msg(TrcDbgTrace.MsgLevel level, String msg)
    {
        switch (level)
        {
            case FATAL:
            case ERR:
                Log.e(TAG, msg);
                break;

            case WARN:
                Log.w(TAG, msg);
                break;

            case INFO:
                Log.i(TAG, msg);
                break;

            case VERBOSE:
                Log.v(TAG, msg);
                break;
        }
    }   //msg

    /**
     * This method is called to print a message to the debug console.
     *
     * @param msg specifies the message.
     */
    public static void traceMsg(String msg)
    {
        Log.d(TAG, msg);
    }   //traceMsg

}   //class HalDbgLog
