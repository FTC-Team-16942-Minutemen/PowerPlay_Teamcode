package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AlignmentSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    DistanceSensor m_distanceSensor;
    DigitalChannel m_beamBreak;

    public AlignmentSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
        m_distanceSensor = hardwareMap.get(DistanceSensor.class,"DistanceSensor");
        m_beamBreak = hardwareMap.get(DigitalChannel.class,"IRBB");
    }

    public double getDistance() {
        return m_distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean getBeamBreakState(){return m_beamBreak.getState();}
    @Override
    public void periodic() {
    }

}
