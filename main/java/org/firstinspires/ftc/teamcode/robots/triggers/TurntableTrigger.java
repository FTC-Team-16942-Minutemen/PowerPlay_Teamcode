package org.firstinspires.ftc.teamcode.robots.triggers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

@Config
public class TurntableTrigger extends Trigger {
    LinearSlideSubsystem m_linearSlideSubsystem;
    public static double positionThreshold = 2000;

    public TurntableTrigger(LinearSlideSubsystem linearSlideSubsystem){
        m_linearSlideSubsystem = linearSlideSubsystem;
    }
    @Override
    public boolean get(){
        if(m_linearSlideSubsystem.getElevatorPosition() > positionThreshold){
            return true;
        } else {
            return false;
        }
    }

}
