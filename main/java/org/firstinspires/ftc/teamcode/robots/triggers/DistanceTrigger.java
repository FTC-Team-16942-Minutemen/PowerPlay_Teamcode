package org.firstinspires.ftc.teamcode.robots.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;

public class DistanceTrigger extends Trigger {

    DistanceSensorSubsystem m_distanceSensorSubsystem;
    double m_distanceThreshold;

    public DistanceTrigger(DistanceSensorSubsystem distanceSensorSubsystem, double distanceThreshold)
    {
        m_distanceSensorSubsystem = distanceSensorSubsystem;
        m_distanceThreshold = distanceThreshold;
    }

    @Override
    public boolean get()
    {
        if(m_distanceSensorSubsystem.getDistance() < m_distanceThreshold)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
