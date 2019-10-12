package org.firstinspires.ftc.teamcode;

public class Navigation {
    final double ENCODER_WHEEL_DIAMETER = 4.8;
    final int ENCODER_COUNTS_PER_ROTATION = 1440;
    final double ROBOT_WHEELBASE_WIDTH = 36.83; //unit in cm
    Position position = new Position();

    public double calculateAngle(double left_encoder, double right_encoder){
        double encoder_difference = convertEncoderCountsToCentimeters( right_encoder - left_encoder);

        return (180 * (encoder_difference))/(Math.PI * ROBOT_WHEELBASE_WIDTH);
    }

    public void findRelativePosition(double leftEncoder, double rightEncoder, double horEncoder, double angle) {
        double x1 = leftEncoder * Math.sin(angle);
        double y1 = leftEncoder * Math.cos(angle);
        double x2 = horEncoder * Math.cos(angle);
        double y2 = horEncoder * Math.sin(angle);

        //double[] coords = {x1 + x2, y1 + y2};
        position.setRelativeX(x1+x2);
        position.setRelativeY(y1-y2);
    }

    //Alejandra, Athena, Marianna update the world position 10/11/19
    public void  calculateWorldX(){
        double pX = position.getWorldPreviousX();
        double rX = position.getRelativeX();
        position.setWorldX(pX + rX);
    }

    public void calculateWorldY () {
        double pY = position.getWorldPreviousY();
        double rY = position.getRelativeY();
        position.setWorldY(pY + rY);
    }

    public void calculateWorldAngle () {
        double pA = position.getWorldPreviousAngle();
        double rA = position.getRelativeAngle();
        position.setWorldAngle(pA + rA);
    }
    // October 6th: Lucas and Marianna
    public double convertEncoderCountsToCentimeters(double encoderCounts) {
        return encoderCounts*((ENCODER_WHEEL_DIAMETER*Math.PI)/ENCODER_COUNTS_PER_ROTATION);
    }

    public void findDisplacement() {

    }
}
