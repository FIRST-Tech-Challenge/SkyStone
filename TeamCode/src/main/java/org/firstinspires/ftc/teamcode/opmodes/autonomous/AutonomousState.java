package org.firstinspires.ftc.teamcode.opmodes.autonomous;


/*
    Represents the current "stage" of the autonomous program
    START: Robot has just been placed. Not moving.
    TO_WALL: Robot is driving directly perpendicular to wall towards wall
    TO_DEPOT: Robot is driving to depot
    DROP_MARKER: Robot is dropping marker in depot
    TO_CRATER: Robot is driving to crater to park
    END: Robot has parked. Not moving
 */
public enum AutonomousState
{
    TO_WALL, TO_DEPOT, DROP_MARKER, TO_CRATER, END;

    @Override
    public String toString()
    {
        switch (this)
        {
            case TO_WALL:
                return "TO_WALL";
            case TO_DEPOT:
                return "TO_DEPOT";
            case DROP_MARKER:
                return "DROP_MARKER";
            case TO_CRATER:
                return "TO_CRATER";
            case END:
                return "END";
            default:
                return "Invalid state";
        }
    }
}
