package org.firstinspires.ftc.teamcode.utilities

import kotlin.math.abs

//fun lerp(min: Double, max: Double, f: Double) = min + f * (max - min)

//fun clamp(x: Double, a: Double, b: Double): Double = if (x < a) a else min(x, b)

fun closestAngleDifference(ang1: Double, ang2: Double): Double {
    val difference = abs(ang2 - ang1)
    val secondDifference = 360 - difference
    val returnVal = if (difference < secondDifference) difference else secondDifference

    val actualDiff = ang2 - ang1
    return when {
        -180 < actualDiff && actualDiff < 0 -> -returnVal
        actualDiff > 180 -> -returnVal
        else -> returnVal
    }

}

fun ClosedFloatingPointRange<Double>.lerp(t: Double): Double {
    return this.start + t * (this.endInclusive - this.start)
}

