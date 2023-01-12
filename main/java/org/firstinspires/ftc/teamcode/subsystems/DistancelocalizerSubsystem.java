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
    DistanceSensor m_distanceSensorForwardA;
//    DistanceSensor m_distanceSensorForwardB;
//    DistanceSensor m_distanceSensorLeft;
    DistanceSensor m_distanceSensorRight;

    Pose2d pose;
    double XOffset = 5.735;
    double YOffset = 7;
    double a;
    double b;
    double c;


//    public DistancelocalizerSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
//        m_hardwareMap = hardwareMap;
//        m_telemetry = telemetry;
//        m_distanceSensorForwardA = hardwareMap.get(DistanceSensor.class, "FrontRightDS");
////        m_distanceSensorForwardB = hardwareMap.get(DistanceSensor.class, "FrontLeftDS");
////        m_distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "");
//        m_distanceSensorRight = hardwareMap.get(DistanceSensor.class, "RightDS");
//
//    }
//    public Pose2d RelocalizeQuadrantOne(double desiredHeading, double heading){
//        b = 180 - (90 + Math.toDegrees(desiredHeading)- Math.toDegrees(heading));
//        a = 180 - (90 + Math.toDegrees(desiredHeading + 90)- Math.toDegrees(heading));
//        return new Pose2d(
//                (Math.sin(b)* m_distanceSensorForwardA.getDistance(DistanceUnit.INCH))/Math.sin(90),
//                )
//
//        return new Pose2d(72 - (m_distanceSensorForwardA.getDistance(DistanceUnit.INCH)) + XOffset,
//                72 - (m_distanceSensorRight.getDistance(DistanceUnit.INCH)+ YOffset));
//
//    }
//    public Pose2d RelocalizeQuadrantTwo(){
//        return new Pose2d(m_distanceSensorForwardB.getDistance(DistanceUnit.INCH)+ XOffset,
//                m_distanceSensorRight.getDistance(DistanceUnit.INCH)+ YOffset);
//    }
    public Pose2d RelocalizeQuadrantThree(){
        return new Pose2d(m_distanceSensorRight.getDistance(DistanceUnit.INCH)+ XOffset,
                m_distanceSensorForwardA.getDistance(DistanceUnit.INCH)+ YOffset);
    }
//    public Pose2d RelocalizeQuadrantFour(){
//        return new Pose2d(m_distanceSensorForwardB.getDistance(DistanceUnit.INCH)+ XOffset,
//                m_distanceSensorRight.getDistance(DistanceUnit.INCH)+ YOffset);
//    }

}
