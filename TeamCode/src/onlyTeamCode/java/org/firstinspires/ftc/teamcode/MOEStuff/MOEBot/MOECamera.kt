package org.firstinspires.ftc.teamcode.MOEStuff.MOEBot

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.os.Environment
import org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.MOEOpMode
import org.firstinspires.ftc.teamcode.utilities.addData
import java.io.File
import java.io.FileInputStream
import java.io.FileNotFoundException

class MOECamera(private val opMode: MOEOpMode) {
    //TODO: Change to camera
    fun getBitmap(): Bitmap? {
        val options = BitmapFactory.Options()

        options.inPreferredConfig = Bitmap.Config.ARGB_8888
//        opMode.telemetry.addData("external", Environment.getExternalStorageDirectory().toString())
        val f = File(Environment.getExternalStorageDirectory().toString() + "/FirstTest/3.jpg")
        if (!f.exists()) {
            opMode.telemetry.addData("File not found")
            opMode.telemetry.addData(f.toString())
            opMode.telemetry.update()
        }
        opMode.telemetry.update()
        var bitmap: Bitmap? = null;
        try {
            bitmap = BitmapFactory.decodeStream(FileInputStream(f), null, options)
        } catch (e: FileNotFoundException) {
            e.printStackTrace()
        }
        return bitmap
    }
}