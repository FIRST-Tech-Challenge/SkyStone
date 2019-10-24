package org.firstinspires.ftc.teamcode.utilities.PurePursuit

//package org.firstinspires.ftc.teamcode.utilities.PurePursuit
//
//import org.firstinspires.ftc.teamcode.OtherStuff.Point
//import java.util.ArrayList
//import kotlin.math.*
//
//
//class MOEPurePursuitPath(var points: List<Point>, private val options: MOEPurePursuitOptions) {
//
//    init {
//        injectPoints()
//        smoothPoints()
//        setMaxVelocities()
//        smoothVelocities()
//    }
//
//    private fun injectPoints() {
//        val newPathing = ArrayList<Point>()
//
//        for (i in 0 until points.size - 1) {
//            val vector = MOEVector(
//                points[i + 1].x - points[i].x,
//                points[i + 1].y - points[i].y
//            )
//
//            val numberOfPointsThatFit = ceil(vector.magnitude / options.spacing).toInt()
//            vector.normalize()
//            vector.multiplyBy(options.spacing)
//
//            val originalPoint = Point(
//                points[i].x,
//                points[i].y
//            )
//            originalPoint.isCriticalPoint = true;
//            newPathing.add(originalPoint)
//
//            for (a in 1 until numberOfPointsThatFit) {
//                newPathing.add(
//                    Point(
//                        points[i].x + vector.getX() * a,
//                        points[i].y + vector.getY() * a
//                    )
//                )
//            }
//        }
//
//        val lastPoint = points[points.size - 1]
//        lastPoint.isCriticalPoint = true
//        newPathing.add(lastPoint)
//        this.points = newPathing
//    }
//
//    private fun smoothPoints() {
//        val newPath = ArrayList<Point>(points.size)
//
//        for (foo in points) {
//            newPath.add(Point(foo.x, foo.y))
//        }
//
//        var change = options.tolerance
//        while (change >= options.tolerance) {
//            change = 0.0
//            for (i in 1 until points.size - 1) {
//                var aux = newPath[i].x
//                newPath[i].x += options.smoothingA * (points[i].x - newPath[i].x) + options.smoothingB * (newPath[i - 1].x + newPath[i + 1].x - 2.0 * newPath[i].x)
//                change += abs(aux - newPath[i].x)
//
//                aux = newPath[i].y
//                newPath[i].y += options.smoothingA * (points[i].y - newPath[i].y) + options.smoothingB * (newPath[i - 1].y + newPath[i + 1].y - 2.0 * newPath[i].y)
//                change += abs(aux - newPath[i].y)
//            }
//        }
//        points = newPath
//    }
//
//    private fun setMaxVelocities() {
//        for (i in 1 until points.size - 1) {
//            val curvature = Point.getCurvatureOfPoints(points[i - 1], points[i], points[i + 1])
//            println("curvature; $curvature")
//            points[i].velocity = min(options.overallMaxVelocity, options.turningConstant / curvature)
//        }
//        points[0].velocity = options.overallMaxVelocity
//        points[points.size - 1].velocity = 0.0
//    }
//
//    private fun smoothVelocities() {
//        var velocity: Double
//        points[points.size - 1].velocity = 0.0
//        for (i in points.size - 2 downTo 0) {
//            val nextVelocity = points[i + 1].velocity
//            val a = options.overallMaxVelocity
//            val distance = points[i].distanceFrom(points[i + 1])
//            val newVelocity = sqrt(nextVelocity.pow(2.0) + 2.0 * a * distance)
//            velocity = min(points[i].velocity, newVelocity)
//            points[i].velocity = velocity
//        }
//    }
//
//    fun getClosestPointIndex(
//        lastKnownPointIndex: Int, currentPoint: Point
//    ): Int {
//        var closestPointIndex = 0
//        var closestDistance = java.lang.Double.MAX_VALUE
//
//        for (i in lastKnownPointIndex - options.lookBack until lastKnownPointIndex + options.lookForward + 1) {
//            if (i >= 0 && i < points.size) {
//                val dist = points[i].distanceFrom(currentPoint)
//                if (dist < closestDistance) {
//                    closestDistance = dist
//                    closestPointIndex = i
//                }
//            }
//        }
//        return closestPointIndex
//    }
//
//
//
//
//    fun getLookaheadPoint(
//        startPoint: Point,
//        endPoint: Point,
//        currentPosition: Point,
//        lookaheadDistance: Double,
//        onLastSegment: Boolean
//    ): Point? {
//        val progress = getCircleLineIntersection(startPoint, endPoint, currentPosition, lookaheadDistance)
//        if (java.lang.Double.isNaN(progress)) {
//            return null
//        }
//        if (onLastSegment) {
//            //TODO: add last segment code
//        }
//        val intersectVector = Point.sub(endPoint, startPoint)
//        val vectorSegment = Point.multiply(intersectVector, progress)
//        return Point.add(startPoint, vectorSegment)
//    }
//
//    fun getLookaheadPointFromPathing(closestPointIndex: Int, currentPosition: Point): Point {
//        for (i in closestPointIndex until pathing.size - 1) {
//            val a = points[i]
//            val b = points[i + 1]
//
//            val lp = getLookaheadPoint(a, b, currentPosition, AllConstants.PurePursuit.LOOKAHEAD_DISTANCE, false)
//
//            if (lp != null) {
//                return lp
//            }
//        }
//        return points[closestPointIndex]
//    }
//    operator fun get(index: Int): Point = points[index]
//     fun getSize() = points.size;
//
//
//
//}