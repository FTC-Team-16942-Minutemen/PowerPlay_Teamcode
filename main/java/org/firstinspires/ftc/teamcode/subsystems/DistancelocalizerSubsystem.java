package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;


import com.acmerobotics.roadrunner.geometry.Pose2d;




public class DistancelocalizerSubsystem extends SubsystemBase{
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    DistanceSensor m_distanceSensorLeft;
    DistanceSensor m_distanceSensorRight;

    double XOffset = 5.735;
    double YOffset = 7;
    double toWallOffset = 6.85;


    public DistancelocalizerSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "LeftDS");
        m_distanceSensorRight = hardwareMap.get(DistanceSensor.class, "RightDS");

    }
//    public Pose2d RelocalizeNorthWall(){
//        return new Pose2d(72 - toWallOffset,
//                72 - (m_distanceSensorRight.getDistance(DistanceUnit.INCH)+ YOffset));
//
//    }
    public Pose2d RelocalizeEastWall(){
        return new Pose2d(-72 + toWallOffset,
                72- (m_distanceSensorRight.getDistance(DistanceUnit.INCH)+ YOffset));
    }
//    public Pose2d RelocalizeSouthWall(){
//        return new Pose2d(m_distanceSensorRight.getDistance(DistanceUnit.INCH)+ XOffset,
//                m_distanceSensorForwardA.getDistance(DistanceUnit.INCH)+ YOffset);
//    }
    public Pose2d RelocalizeWestWall(){
        return new Pose2d(72- XOffset,
                m_distanceSensorLeft.getDistance(DistanceUnit.INCH)+ YOffset);
    }

}
