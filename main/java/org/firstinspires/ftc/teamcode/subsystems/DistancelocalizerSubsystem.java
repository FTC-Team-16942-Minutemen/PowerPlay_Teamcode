package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;




public class DistancelocalizerSubsystem extends SubsystemBase{
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    DistanceSensor m_distanceSensorForwardA;
    DistanceSensor m_distanceSensorForwardB;
    DistanceSensor m_distanceSensorLeft;
    DistanceSensor m_distanceSensorRight;
    Pose2d pose;
    double XOffset = 5;
    double YOffset = 5;



    public DistancelocalizerSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
//        m_distanceSensorForwardA = hardwareMap.get(DistanceSensor.class, "");
//        m_distanceSensorForwardB = hardwareMap.get(DistanceSensor.class, "");
//        m_distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "");
//        m_distanceSensorRight = hardwareMap.get(DistanceSensor.class, "");

    }
    public Pose2d RelocalizeQuadrantOne(){
        return new Pose2d((m_distanceSensorForwardA.getDistance(DistanceUnit.INCH)) + XOffset,
                m_distanceSensorRight.getDistance(DistanceUnit.INCH)+ YOffset);
    }
    public Pose2d RelocalizeQuadrantTwo(){
        return new Pose2d(m_distanceSensorForwardB.getDistance(DistanceUnit.INCH)+ XOffset,
                m_distanceSensorRight.getDistance(DistanceUnit.INCH)+ YOffset);
    }
    public Pose2d RelocalizeQuadrantThree(){
        return new Pose2d(m_distanceSensorForwardB.getDistance(DistanceUnit.INCH)+ XOffset,
                m_distanceSensorRight.getDistance(DistanceUnit.INCH)+ YOffset);
    }
    public Pose2d RelocalizeQuadrantFour(){
        return new Pose2d(m_distanceSensorForwardB.getDistance(DistanceUnit.INCH)+ XOffset,
                m_distanceSensorRight.getDistance(DistanceUnit.INCH)+ YOffset);
    }

}
