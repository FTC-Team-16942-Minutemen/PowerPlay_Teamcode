package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class ParkingCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;

    public ParkingCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem)
    {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize()
    {
        if (m_visionSubsystem.getParkingSpot() == 0)//magenta
        {
            m_driveSubsystem.turnAsync(-3.1415926/2.0);
        }
        else if (m_visionSubsystem.getParkingSpot() == 1)
        {
            m_driveSubsystem.turnAsync(3.1415926/2.0);
        }
        else if (m_visionSubsystem.getParkingSpot() == 2)
        {
            m_driveSubsystem.turnAsync(3.1415926);
        }
    }

    @Override
    public void execute()
    {
        m_driveSubsystem.update();
    }

    @Override
    public boolean isFinished()
    {
        return Thread.currentThread().isInterrupted() || !m_driveSubsystem.isBusy();
    }

    @Override
    public void end(boolean interrupted)
    {
        if(interrupted)
        {
            m_driveSubsystem.stop();
        }
    }
}
