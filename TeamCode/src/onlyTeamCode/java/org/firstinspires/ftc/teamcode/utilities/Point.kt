package org.firstinspires.ftc.teamcode.OtherStuff

import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.roundToInt
import kotlin.math.sqrt

class Point(var x: Double, var y: Double) {
    var prefixDistance: Double = 0.toDouble()
    var velocity: Double = 0.toDouble()
    var isCriticalPoint: Boolean = false

    val xAsInt: Int
        get() = x.roundToInt()

    val yAsInt: Int
        get() = y.roundToInt()

    init {
        this.isCriticalPoint = false
    }

    //
//    fun setCriticalPoint(val value: Boolean) {
//        this.isCriticalPoint = value
//    }
    override fun toString(): String {
        return "Point [x=$x, y=$y, velocity=$velocity]"
    }

    fun simpleString(): String {
        return x.toString() + "\t" + y
    }

    fun dot(p: Point): Double {
        return x * p.x + y * p.y
    }

    fun distanceFrom(point: Point): Double {
        return sqrt((this.x - point.x).pow(2.0) + (this.y - point.y).pow(2.0))
    }

    companion object {

        fun sub(a: Point, b: Point): Point {
            return Point(a.x - b.x, a.y - b.y)
        }

        fun multiply(a: Point, n: Double): Point {

            return Point(a.x * n, a.y * n)
        }

        fun add(a: Point, b: Point): Point {
            return Point(a.x + b.x, a.y + b.y)
        }

        fun angleBetweenPoints(a: Point, b: Point): Double {
            var angle = Math.toDegrees(atan2(b.y - a.y, b.x - a.x))

            if (angle < 0) {
                angle += 360.0
            }

            return angle
        }

        fun getCurvatureOfPoints(leftPoint: Point, centerPoint: Point, rightPoint: Point): Double {
            if (centerPoint.x == 90.0) {
                centerPoint.x += 0.0001
            } //Fixing for DivideByZero error

            val k_1 =
                    0.5 * (centerPoint.x * centerPoint.x + centerPoint.y * centerPoint.y - leftPoint.x * leftPoint.x - leftPoint.y * leftPoint.y) / (centerPoint.x - leftPoint.x)
            val k_2 = (centerPoint.y - leftPoint.y) / (centerPoint.x - leftPoint.x)
            val b =
                    0.5 * (leftPoint.x * leftPoint.x - 2.0 * leftPoint.x * k_1 + leftPoint.y * leftPoint.y - rightPoint.x * rightPoint.x + 2.0 * rightPoint.x * k_1 - rightPoint.y * rightPoint.y) / (rightPoint.x * k_2 - rightPoint.y + leftPoint.y - leftPoint.x * k_2)
            val a = k_1 - k_2 * b
            val r = Math.sqrt(Math.pow(centerPoint.x - a, 2.0) + Math.pow(centerPoint.y - b, 2.0))
            val curvature = 1 / r
            return if (curvature.isNaN()) {
                0.0
            } else curvature
        }
    }

}
