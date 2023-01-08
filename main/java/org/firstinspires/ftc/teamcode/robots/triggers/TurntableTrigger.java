package org.firstinspires.ftc.teamcode.robots.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurntableSubsystem;

public class TurntableTrigger extends Trigger {
    LinearSlideSubsystem m_linearslideSubsystem;
    

    public TurntableTrigger(LinearSlideSubsystem linearSlideSubsystem){
        linearSlideSubsystem = m_linearslideSubsystem;
    }
    @Override
    public boolean get(){
        if(m_linearslideSubsystem.getElevatorPosition() > 1000){
            return true;
        } else {
            return false;
        }
    }
}
