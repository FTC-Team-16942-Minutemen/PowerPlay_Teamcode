package org.firstinspires.ftc.teamcode.robots;

public final class Constants
{
    // enum to specify opMode type
    public enum OpModeType {
        TELEOP,
        BLUE_RIGHT_AUTO,
        BLUE_LEFT_AUTO,
        RED_RIGHT_AUTO,
        RED_LEFT_AUTO
    }

    public enum LinearSlideState {
        JUNCTIONLEVEL,
        SCORING,
        ACQUIRED,
        STACKLEVEL,
        GROUNDLEVEL
    }

    public enum OperatorMode {
        SINGLE_OPERATOR_MODE,
        DOUBLE_OPERATOR_MODE
    }
}
