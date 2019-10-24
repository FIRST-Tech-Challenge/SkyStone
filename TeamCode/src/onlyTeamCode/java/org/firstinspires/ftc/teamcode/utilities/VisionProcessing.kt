package org.firstinspires.ftc.teamcode.utilities

import android.graphics.Bitmap
import android.graphics.Color
import android.os.Environment
import kotlin.math.roundToInt

const val NUM_BLOCKS: Int = 2

enum class SkyStoneLocation {
    LEFT,
    MIDDLE,
    RIGHT
}

fun getSkyStoneLocationsFromBitmap(bm: Bitmap): SkyStoneLocation {
    val image = bm.scale(2, 1)
    image!!.saveTo(Environment.getExternalStorageDirectory().absolutePath + "/FirstTest/1_crop_crop.jpg")
    val leftBound = 0
    val middleBound = (image.width.toFloat() / NUM_BLOCKS).roundToInt()
    val rightBound = image.width
    val leftSide = getValidPixels(leftBound, middleBound, image)
    val rightSide = getValidPixels(middleBound, rightBound, image)
    assert(leftSide + rightSide > 0)
    return if (leftSide == 1) {
        if (rightSide == 1) {
            SkyStoneLocation.LEFT
        } else {
            SkyStoneLocation.MIDDLE
        }
    } else {
        SkyStoneLocation.RIGHT
    }
}

private fun getValidPixels(leftBound: Int, rightBound: Int, image: Bitmap): Int {
    var pixels = 0
    for (x in leftBound until rightBound) {
        for (y in 0 until image.height) {
            val rgb = image.getPixel(x, y)
            val hsv = FloatArray(3)
            Color.colorToHSV(rgb, hsv)

            if (hsv[1] > 0.4 && hsv[2] > 0.6 && hsv[0] < 60 && hsv[0] > 30) {
                pixels++
            }
        }
    }
    return pixels
}
