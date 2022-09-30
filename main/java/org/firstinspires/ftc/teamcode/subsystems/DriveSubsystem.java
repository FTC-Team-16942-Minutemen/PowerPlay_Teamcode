package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.robots.PowerPlayBot;

import java.util.List;

/**
 * A subsystem that uses the {@link SampleMecanumDrive} class.
 * This periodically calls {@link SampleMecanumDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class DriveSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    SampleMecanumDrive m_drive;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
        m_drive=new SampleMecanumDrive(m_hardwareMap, m_telemetry);
    }



    public void drive(double leftX, double leftY, double rightX, boolean isFieldCentric)
    {
        Pose2d poseEstimate = getPoseEstimate();
        Vector2d input= new Vector2d(leftY, leftX).rotated(
                isFieldCentric ? -poseEstimate.getHeading() :0
        );

        m_drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        rightX
                )
        );
    }

    public void update()
    {
        m_drive.update();
    }

    public boolean isBusy() {
        return m_drive.isBusy();
    }

    public void stop(){
        drive(0,0,0,true);
    }

    public void followTrajectory(Trajectory trajectory){
        m_drive.followTrajectory(trajectory);
    }

    public void followTrajectoryAsync(Trajectory trajectory) { m_drive.followTrajectoryAsync(trajectory);}

    public void turnAsync(double angle) {m_drive.turnAsync(angle);}

    public void setPoseEstimate(Pose2d pose)
    {
        m_drive.setPoseEstimate(pose);
    }

    public Pose2d getPoseEstimate()
    {
        return m_drive.getPoseEstimate();
    }
    public void updatePoseEstimate()
    {
        m_drive.updatePoseEstimate();
    }

}
