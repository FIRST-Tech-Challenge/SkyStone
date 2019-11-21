package org.firstinspires.ftc.robotlib.information;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotlib.Constants;
import org.firstinspires.ftc.robotlib.navigation.Point3D;

/**
 * Stores the location info for the autonomous robot
 */
public class LocationInfo {
    // This point is the most accurate, but only works when a trackable is visible
    private OpenGLMatrix lastRobotLocation;
    private long lastRobotLocationTime;

    private OpenGLMatrix robotLocation;
    private OpenGLMatrix lastRobotLocationFromSkystone;

    public LocationInfo() {
        lastRobotLocation = null;
        robotLocation = null;
        lastRobotLocationFromSkystone = null;
    }

    /**
     * Retrieves the robot location (based on encoder values and tracking)
     * @return 3D point
     */
    public Point3D getRobotLocation() {
        return this.convertOpenGLMatrix(robotLocation);
    }

    /**
     * Retrieves the robot location
     * @return Raw OpenGLMatrix
     */
    public OpenGLMatrix getRobotLocationMatrix() {
        return robotLocation;
    }

    /**
     * Retrieves the last location known (for certain) based on tracking
     * @return 3D point
     */
    public Point3D getLastRobotLocation() {
        return this.convertOpenGLMatrix(lastRobotLocation);
    }

    /**
     * Retrieves the last location relative to a Skystone
     * @return 3D point
     */
    public Point3D getLastRobotLocationFromSkystone() {
        return this.convertOpenGLMatrix(lastRobotLocationFromSkystone);
    }

    /**
     * Translates the local robot location
     * @param dx x translation
     * @param dy y translation
     * @param dz z translation (should not change)
     */
    public void translateRobotLocation(float dx, float dy, float dz) {
        if (this.robotLocation == null) return; // silently fails..... ehh
        this.robotLocation.translate(dx, dy, dz);
    }

    /**
     * Translates the local robot location based on a course and distance
     * @param heading Heading/Yaw
     * @param distance Distance traveled
     */
    public void translateRobotLocation(double heading, double distance) {
        this.translateRobotLocation((float) (distance * Math.cos(heading)), (float) (distance * Math.sin(heading)), 0f);
    }

    /**
     * Sets the last robot location which should be provided by a trackable
     * @param lastRobotLocation new location as an OpenGLMatrix
     */
    public void setLastRobotLocation(OpenGLMatrix lastRobotLocation) {
        this.lastRobotLocation = lastRobotLocation;
        this.lastRobotLocationTime = System.currentTimeMillis();
        this.robotLocation = this.lastRobotLocation;
    }

    /**
     * Retrieves the timestamp of the last tracked trackable
     * @return Milliseconds (Unix Epoch)
     */
    public long getLastRobotLocationTime() {
        return lastRobotLocationTime;
    }

    /**
     * Sets the last robot location provided by a Skystone
     * @param lastRobotLocationFromSkystone position relative to Skystone
     */
    public void setLastRobotLocationFromSkystone(OpenGLMatrix lastRobotLocationFromSkystone) {
        this.lastRobotLocationFromSkystone = lastRobotLocationFromSkystone;
    }

    /**
     * Converts an OpenGLMatrix to a 3D point (assuming the values are in millimeters)
     * @param openGLMatrix matrix to convert
     * @return 3D point
     */
    private Point3D convertOpenGLMatrix(OpenGLMatrix openGLMatrix) {
        VectorF translation = openGLMatrix.getTranslation();
        return new Point3D(translation.get(0) / Constants.mmPerInch, translation.get(1) / Constants.mmPerInch, translation.get(2) / Constants.mmPerInch);
    }
}
