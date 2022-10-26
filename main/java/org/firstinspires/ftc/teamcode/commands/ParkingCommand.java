package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class ParkingCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private Trajectory m_trajectorySpot1;
    private Trajectory m_trajectorySpot0;

    public ParkingCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem)
    {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_trajectorySpot1 = AssetsTrajectoryManager.load("BlueLeftParking1");
        m_trajectorySpot0 = AssetsTrajectoryManager.load("BlueLeftParking0");

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize()
    {
        if (m_visionSubsystem.getParkingSpot() == 0)//magenta
        {
            m_driveSubsystem.followTrajectoryAsync(m_trajectorySpot0);
        }
        else if (m_visionSubsystem.getParkingSpot() == 1)
        {
            m_driveSubsystem.followTrajectoryAsync(m_trajectorySpot1);
        }
        else if (m_visionSubsystem.getParkingSpot() == 2)
        {

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
