package org.firstinspires.ftc.teamcode.util.math;

import java.lang.Math;

public class Angle {
	// angle in degrees
	private final double angle;

	/**
	 * @param angle the angle in degrees
	 */
	public Angle(double angle) {
		this.angle = angle;
	}

	/**
	 * @return angle in radians
	 */
	public double radians() {
		return Math.toRadians(this.angle);
	}

	/**
	 * @return angle in degrees
	 */
	public double degrees() {
		return this.angle;
	}
}
