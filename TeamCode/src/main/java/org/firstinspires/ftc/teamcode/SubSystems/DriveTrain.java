package org.firstinspires.ftc.teamcode.SubSystems;


/** Definition of Robot Chassis.
 *  Chassis has :
 *      4 DC motors connected to Mecanum wheels
 *      1 limit switch on left front bumper to identify hitting to foundation plate or other walls
 *      2 Color sensors pointing down one on left and another on right
 *      (to identify red / blue lines below skybridge for parking
 *
 *      Robot 1 : Chassis Motor : 5201 Series, 26:1 Ratio, 210 RPM Spur Gear Motor w/Encoder
 *      Encoder Countable Events Per Revolution (Output Shaft)	723.24 (Rises & Falls of Ch A & B)
 *
 *      Robot 2 : 5202 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 312 RPM, 3.3 - 5V Encoder)
 *      Encoder Countable Events Per Revolution (Output Shaft)	537.6 (Rises & Falls of Ch A & B)
 *
 * @ChassisMethods : Chassis(HardwareMap) - Constructor
 * @ChassisMethods : initChassis()
 * @ChassisMethods : configureChassis()
 * @ChassisMethods : resetChassis()
 * @ChassisTeleOpMethods : runByGamepadCommand()
 * @ChassisAutoMethods : runDistance()
 * @ChassisAutoMethods : runFwdTill_frontleftChassisTouchSensor_Pressed(
 * @ChassisAutoMethods : runTill_ChassisRightColorSensorIsRed()
 * @ChassisAutoMethods : turnRobotByAngle()
 * @ChassisAutoMethods : resetColorSensorEnabled()
 * @ChassisAutoMethods : leftColorSensorIsRed()
 * @ChassisAutoMethods : rightColorSensorIsBlue()
 * @ChassisAutoMethods : leftColorSensorIsBlue()
 * @ChassisAutoMethods : rightColorSensorIsRed()
 * @ChassisAutoMethods : frontleftBumperSensorIsPressed()
 *
 */
/*public abstract class DriveTrain {
    public DriveTrain() {
        super();
    }
}*/
