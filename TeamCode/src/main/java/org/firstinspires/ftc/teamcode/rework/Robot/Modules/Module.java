package org.firstinspires.ftc.teamcode.rework.Robot.Modules;


import org.firstinspires.ftc.teamcode.rework.Robot.ReworkRobot;

public interface Module {

    /**
     * Initializes the module. This includes setting up all motors/servos
     * */
    public void init(ReworkRobot robot);

    /**
     * Updates the module, executing all the tasks it should complete on every iteration,
     * utilizing the module's states. This separate method allows for each module to be updated
     * on a different thread from where the states are set.
     */
    public void update();


}
