package org.firstinspires.ftc.teamcode.utilities.PurePursuit

data class MOEPurePursuitOptions(
        val overallMaxVelocity: Double,
        val spacing: Double,
        val tolerance: Double,
        val smoothingA: Double,
        val smoothingB: Double,
        val turningConstant: Double,
        val lookBack: Int,
        val lookForward: Int
) {

}
