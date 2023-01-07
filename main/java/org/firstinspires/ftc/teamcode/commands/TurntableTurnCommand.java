package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.TurntableSubsystem;

import java.util.function.BooleanSupplier;

public class TurntableTurnCommand extends CommandBase{
    TurntableSubsystem m_turntableSubsystem;
    GamepadEx gamepadEx;
    BooleanSupplier BButtonSupplier;

    public TurntableTurnCommand(TurntableSubsystem turntableSubsystem, BooleanSupplier BButtonSupplier) {
        turntableSubsystem = m_turntableSubsystem;
        BButtonSupplier = BButtonSupplier;
    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        if(m_turntableSubsystem.getTurntablePosition() > 0.0){
            m_turntableSubsystem.faceForward();
        } else {
            m_turntableSubsystem.faceBackwards();
        }
    }
}
