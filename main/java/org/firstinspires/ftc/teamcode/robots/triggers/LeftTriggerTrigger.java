package org.firstinspires.ftc.teamcode.robots.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

public class LeftTriggerTrigger extends Trigger {

    LinearSlideSubsystem m_linearSlideSubsystem;
    double m_inputThreshold;

    public LeftTriggerTrigger(LinearSlideSubsystem linearSlideSubsystem, double inputThreshold)
    {
        m_linearSlideSubsystem = linearSlideSubsystem;
        m_inputThreshold = inputThreshold;
    }

    @Override
    public boolean get()
    {
        if(m_linearSlideSubsystem.getElevatorPosition() > m_inputThreshold)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
