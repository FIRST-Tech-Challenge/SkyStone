//package org.firstinspires.ftc.teamcode.utilities.PurePursuit
//
//import org.firstinspires.ftc.teamcode.OtherStuff.Point
//
//class MOEPurePursuitSystem(points: List<Point>, options: MOEPurePursuitOptions) {
//    private var path: MOEPurePursuitPath = MOEPurePursuitPath(points, options)
//
//    fun newRunPathing(currentPosition: Point,currentHeading:Int) {
//
//        var lastKnownPointIndex = 0
//        var closestPoint: Point
//        do {
//
//            lastKnownPointIndex = path.getClosestPointIndex(
//                lastKnownPointIndex,
//                currentPosition
//            )
//            closestPoint = path[lastKnownPointIndex]
//            //
////            if (closestPoint.isCriticalPoint && lastKnownPointIndex != path.getSize() - 1) {
////                val angleToTurn =
////                    Point.angleBetweenPoints(currentPosition, getNextClosestPoint(lastKnownPointIndex, path))
////                //            robot.opMode.
////
////                robot.chassis.turnToDegrees(angleToTurn, true)
////                closestPoint.isCriticalPoint = false
////                continue
////            }
//
//            val heading = currentHeading
//
//            val lookaheadPoint = path.getLookaheadPointFromPathing(pursuitPathing, lastKnownPointIndex, currentPosition)
//            val curvature = getSignedCurvatureFromLookaheadPoint(
//                lookaheadPoint,
//                currentPosition,
//                heading,
//                AllConstants.PurePursuit.LOOKAHEAD_DISTANCE
//            )
//            //
//            val leftWheelTargetVelocity = getLeftWheelTargetVelocity(closestPoint.velocity, curvature).toInt()
//            val rightWheelTargetVelocity = getRightWheelTargetVelocity(closestPoint.velocity, curvature).toInt()
//            //            leftWheelTargetVelocity = normalizeVelocity(leftWheelTargetVelocity);
//            //            rightWheelTargetVelocity = normalizeVelocity(rightWheelTargetVelocity);
//            //            double[] velocities = normalizeVelocities(leftWheelTargetVelocity, rightWheelTargetVelocity);
//            //            leftWheelTargetVelocity = velocities[0];
//            //            rightWheelTargetVelocity = velocities[1];
//            //robot.chassis.setVelocities(leftWheelTargetVelocity,rightWheelTargetVelocity);
//            //TODO: if (!path.isForward()) {
//            //            leftTargetVel = -leftTargetVel;
//            //            rightTargetVel = -rightTargetVel;
//
//            //            double leftFeedback = getWheelFeedbackVelocity(leftWheelTargetVelocity,
//            //                                                           robot.chassis.getAStarVelocity(robot.chassis.frontLeftMotor));
//            //            double rightFeedback = getWheelFeedbackVelocity(rightWheelTargetVelocity,
//            //                                                            robot.chassis.getAStarVelocity(robot.chassis.frontRightMotor));
//            val leftFeedback = 0
//            val rightFeedback = 0
//
//            val leftPower = leftWheelTargetVelocity + leftFeedback
//            val rightPower = rightWheelTargetVelocity + rightFeedback
//            val scaleDown = 0.4
//            val scaledLeftVelocity = Math.round(leftPower * AllConstants.Tics.FORWARD_ASTAR_TICS * scaleDown) as Int
//            val scaledRightVelocity = Math.round(rightPower * AllConstants.Tics.FORWARD_ASTAR_TICS * scaleDown) as Int
//            val scaledMaxVelocity =
//                Math.round(AllConstants.PurePursuit.OVERALL_MAX_VELOCITY * AllConstants.Tics.FORWARD_ASTAR_TICS * scaleDown) as Int
//            //            robot.tts.speak(String.valueOf(scaledMaxVelocity), TextToSpeech.QUEUE_ADD, null, "");
//            //            scaledLeftVelocity = Math.min(scaledLeftVelocity,scaledMaxVelocity);
//            //            scaledRightVelocity = Math.min(scaledRightVelocity,scaledMaxVelocity);
//
//            robot.chassis.setVelocity(scaledLeftVelocity, scaledRightVelocity)
//
//            //robot.chassis.backLeftMotor
//            //            robot.telemetry.addData("coords: ", currentPosition.x + ", " + currentPosition.y);
//            //            robot.telemetry.addData("lp: ", lookaheadPoint.toString());
//            //
//            //            robot.telemetry.addData("left:", leftWheelTargetVelocity);
//            //            robot.telemetry.addData("right:", rightWheelTargetVelocity);
//            //            robot.telemetry.update();
//            val leftWheelTargetVelocity1 = getLeftWheelTargetVelocity(closestPoint.velocity, curvature)
//            val rightWheelTargetVelocity1 = getRightWheelTargetVelocity(closestPoint.velocity, curvature)
//
//            robot.showTelemetry(
//                "location",
//                currentPosition.simpleString(),
//                "angle",
//                "" + heading,
//                "lookAhead",
//                lookaheadPoint.simpleString(),
//                "curvature",
//                "" + curvature,
//                "leftWheelTarget",
//                "" + scaledLeftVelocity,
//                "rightWheelTarget",
//                "" + scaledRightVelocity,
//                "forward Velocity",
//                (leftWheelTargetVelocity1 + rightWheelTargetVelocity1) / 2 + "",
//                "angular Velocity",
//                (leftWheelTargetVelocity1 - rightWheelTargetVelocity1) / AllConstants.PurePursuit.TRACK_WIDTH + ""
//            )
//            while (robot.opMode.opModeIsActive() && robot.opMode.gamepad1.a) {
//                robot.chassis.setVelocity(0)
//                robot.showTelemetry(
//                    "location",
//                    currentPosition.simpleString(),
//                    "angle",
//                    "" + heading,
//                    "lookAhead",
//                    lookaheadPoint.simpleString(),
//                    "curvature",
//                    "" + curvature,
//                    "leftWheelTarget",
//                    "" + scaledLeftVelocity,
//                    "rightWheelTarget",
//                    "" + scaledRightVelocity,
//                    "forward Velocity",
//                    (leftWheelTargetVelocity1 + rightWheelTargetVelocity1) / 2 + "",
//                    "angular Velocity",
//                    (leftWheelTargetVelocity1 - rightWheelTargetVelocity1) / AllConstants.PurePursuit.TRACK_WIDTH + ""
//                )
//            }
//        } while (lastKnownPointIndex < pursuitPathing.size - 1 && robot.opMode.opModeIsActive()) //TODO: use robot x & robot y for more accurate ending
//    }
//
//}