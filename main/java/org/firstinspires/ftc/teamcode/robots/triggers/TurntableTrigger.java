package org.firstinspires.ftc.teamcode.robots.triggers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

@Config
public class TurntableTrigger extends Trigger {
    LinearSlideSubsystem m_linearSlideSubsystem;
    public static double goingUpPositionThreshold = 1000;
    public static double goingDownPositionThreshold = 2500;
    public TurntableTrigger(LinearSlideSubsystem linearSlideSubsystem){
        m_linearSlideSubsystem = linearSlideSubsystem;
    }
    @Override
    public boolean get() {
        if (m_linearSlideSubsystem.getElevatorPosition() < m_linearSlideSubsystem.getPositionSetpoint()){
            if(m_linearSlideSubsystem.getElevatorPosition() > goingUpPositionThreshold){
                return true;
            }
            else{
                return false;
            }
        } //runs this when the elevator is going up
         else {
            if(m_linearSlideSubsystem.getElevatorPosition() > goingDownPositionThreshold){
                return true;
            }else{
                return false;
            }
        }

    }

}
