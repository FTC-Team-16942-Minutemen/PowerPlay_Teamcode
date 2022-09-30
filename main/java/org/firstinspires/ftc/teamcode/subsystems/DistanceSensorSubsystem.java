package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    DistanceSensor m_distanceSensor;

    public DistanceSensorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
        m_distanceSensor = hardwareMap.get(DistanceSensor.class,"DistanceSensor");

    }

    public double getDistance() {
        double distance = m_distanceSensor.getDistance(DistanceUnit.INCH);
        m_telemetry.addData("Distance: ", distance);
        m_telemetry.update();
        return distance;
    }


}
