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
    DistanceSensor m_distanceSensorA;
    DistanceSensor m_distanceSensorB;
    DigitalChannel m_beamBreak;

    public AlignmentSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
        m_distanceSensorA = hardwareMap.get(DistanceSensor.class,"DistanceSensor");
        m_distanceSensorB = hardwareMap.get(DistanceSensor.class,"DistanceSensorB");
        m_beamBreak = hardwareMap.get(DigitalChannel.class,"IRBB");
    }

    public double getDistanceDifference() {
        return m_distanceSensorA.getDistance(DistanceUnit.INCH) - m_distanceSensorB.getDistance(DistanceUnit.INCH);
    }

    public boolean getBeamBreakState(){return m_beamBreak.getState();}
    @Override
    public void periodic() {
//        m_telemetry.addData("BB:", m_beamBreak.getState());
//        m_telemetry.update();
    }

}
