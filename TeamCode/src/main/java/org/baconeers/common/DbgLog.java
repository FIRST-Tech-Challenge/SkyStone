/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.baconeers.common;

/**
 * Provide utility methods for debug logging
 */
public class DbgLog {
    private DbgLog() {}

    /**
     * Tag used by logcat
     */
    public static final String TAG = "FIRST";

    public static final String ERROR_PREPEND = "### ERROR: ";

    /**
     * Log a debug message
     * @param message
     */
    public static void msg(String message) {
        android.util.Log.i(TAG, message);
    }

    public static void msg(String format, Object... args) {
        msg(String.format(format, args));
    }


    /**
     * Log an error message
     * <p>
     * Messages will be prepended with the ERROR_PREPEND string
     * @param message
     */
    public static void error(String message) {
        android.util.Log.e(TAG, ERROR_PREPEND + message);
    }

    public static void error(String format, Object... args) {
        error(String.format(format, args));
    }

    public static void logStacktrace(Exception e) {
        msg(e.toString());
        for (StackTraceElement el : e.getStackTrace()) {
            msg(el.toString());
        }
    }
}