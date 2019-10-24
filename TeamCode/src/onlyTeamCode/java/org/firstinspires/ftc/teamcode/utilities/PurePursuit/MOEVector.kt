package org.firstinspires.ftc.teamcode.utilities.PurePursuit

import kotlin.math.sqrt

class MOEVector(private var x: Double, private var y: Double) {
    var magnitude: Double = 0.toDouble()
        private set

    init {
        updateMagnitude()
    }

    fun setX(x: Double) {
        this.x = x
        updateMagnitude()
    }

    fun setY(y: Double) {
        this.y = y
        updateMagnitude()
    }

    fun getX(): Double {
        return x
    }

    fun getY(): Double {
        return y
    }

    fun updateMagnitude() {
        this.magnitude = sqrt(x * x + y * y)
    }

    fun normalize() {
        this.x /= this.magnitude
        this.y /= this.magnitude
    }

    fun multiplyBy(scalar: Double) {
        this.x *= scalar
        this.y *= scalar
    }

    companion object {

        fun addVectors(a: MOEVector, b: MOEVector): MOEVector {
            return MOEVector(a.x + b.x, a.y + b.y)
        }

        fun subtractVectors(a: MOEVector, b: MOEVector): MOEVector {
            return MOEVector(a.x - b.x, a.y - b.y)
        }
    }
}
