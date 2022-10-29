package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.subsystems.ClawIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.lang.ref.PhantomReference;
import java.util.function.DoubleSupplier;

public class ScoringCommand extends CommandBase {
    private final ClawIntakeSubsystem m_clawSubsystem;
    private final LinearSlideSubsystem m_linearSlideSubsystem;
    private int m_currentSetPoint;
    private double m_currentPower;
    private double PERCENT_DROP = 0.8;

    private final DoubleSupplier m_leftTriggerSupplier;


    public ScoringCommand(ClawIntakeSubsystem clawSubsystem,
                          LinearSlideSubsystem slideSubsystem,
                          DoubleSupplier leftTriggerSupplier)
    {
        m_clawSubsystem = clawSubsystem;
        m_linearSlideSubsystem = slideSubsystem;
        m_leftTriggerSupplier = leftTriggerSupplier;

        addRequirements(m_clawSubsystem, m_linearSlideSubsystem);
    }

    @Override
    public void initialize()
    {
       m_currentSetPoint= m_linearSlideSubsystem.getPositionSetpoint();
       m_currentPower = m_linearSlideSubsystem.getElevatorPower();
       double newSetPoint = m_currentSetPoint * PERCENT_DROP;
       if(newSetPoint < 0.0)
       {
           newSetPoint = 0.0;
       }
       m_linearSlideSubsystem.setPositionSetPoint((int)newSetPoint);
    }

    @Override
    public void execute()
    {
        m_linearSlideSubsystem.setElevatorPower(m_leftTriggerSupplier.getAsDouble());
    }


    @Override
    public boolean isFinished()
    {
        return Thread.currentThread().isInterrupted() || !m_linearSlideSubsystem.isLinearSlideBusy();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_clawSubsystem.open();
        m_linearSlideSubsystem.setPositionSetPoint(m_currentSetPoint);
        m_linearSlideSubsystem.setElevatorPower(m_currentPower);
//        if(interrupted)
//        {
//        }
    }
}
