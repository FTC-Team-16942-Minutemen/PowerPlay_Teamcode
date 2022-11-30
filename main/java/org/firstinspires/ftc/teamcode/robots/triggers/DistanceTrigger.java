package org.firstinspires.ftc.teamcode.robots.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.AlignmentSubsystem;

public class DistanceTrigger extends Trigger {

    AlignmentSubsystem m_alignmentSubsystem;
    double m_distanceThreshold;

    public DistanceTrigger(AlignmentSubsystem alignmentSubsystem, double distanceThreshold)
    {
        m_alignmentSubsystem = alignmentSubsystem;
        m_distanceThreshold = distanceThreshold;
    }

    @Override
    public boolean get()
    {
        if(m_alignmentSubsystem.getDistanceDifference() > m_distanceThreshold)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
