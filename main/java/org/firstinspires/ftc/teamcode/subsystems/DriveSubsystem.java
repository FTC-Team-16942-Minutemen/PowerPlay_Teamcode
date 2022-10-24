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
import org.ejml.dense.row.mult.VectorVectorMult_CDRM;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.robots.PowerPlayBot;

import java.util.List;
import java.util.Vector;

/**
 * A subsystem that uses the {@link SampleMecanumDrive} class.
 * This periodically calls {@link SampleMecanumDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class DriveSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    SampleMecanumDrive m_drive;
    int[] junctionX = {-48,-24,0, 24, 48};
    int[] junctionY = {-48,-24,0, 24,48};
    double REPULSERADIUS = 9.0;
    double PF_SCALE = 1.0;
    double THROTTLEMINLEVEL = 0.4;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialPose)
    {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
        m_drive=new SampleMecanumDrive(m_hardwareMap, m_telemetry);
        m_drive.setPoseEstimate(initialPose);
    }

    public void drive(double leftX, double leftY, double rightX, double throttle, boolean isFieldCentric)
    {
        Pose2d poseEstimate = getPoseEstimate();
        Vector2d input_vec = new Vector2d(leftY, leftX).rotated(
                isFieldCentric ? -poseEstimate.getHeading() :0
        );

//        Vector2d CorrectedInput = quadraticControlLaw(input_vec);
//        Vector2d CorrectedInput = potentialFields(input_vec);
        Vector2d CorrectedInput = input_vec;

        double throttleSlope = 1 - THROTTLEMINLEVEL;
        double throttleScale = throttleSlope * throttle + THROTTLEMINLEVEL;
        m_drive.setWeightedDrivePower(
                new Pose2d(
                        CorrectedInput.getX() * throttleScale,
                        CorrectedInput.getY() * throttleScale,
                        rightX * throttleScale
                )
        );
    }

    private Vector2d quadraticControlLaw(Vector2d inputVec)
    {
        double outX = inputVec.getX() * inputVec.getX();
        double outY = inputVec.getY() * inputVec.getY();
        return new Vector2d(outX, outY);
    }

    private Vector2d potentialFields(Vector2d input_vec) {
        Vector2d updated_input = input_vec;
        Pose2d poseEstimate = getPoseEstimate();

        for (int ix = 0; ix < junctionX.length; ix++) {
            for (int iy = 0; iy < junctionY.length; iy++) {
                Vector2d r_vec = new Vector2d(poseEstimate.getX(), poseEstimate.getY());
                Vector2d j_vec = new Vector2d(junctionX[ix], junctionY[iy]);
                Vector2d repulse_vec = r_vec.minus(j_vec);

                double dist = repulse_vec.norm();

                if (dist < REPULSERADIUS) {
                    Vector2d unit_repulse_vec = repulse_vec.div(dist);
                    Vector2d new_vector = unit_repulse_vec.times(input_vec.norm() * PF_SCALE);
                    updated_input = input_vec.plus(new_vector);
                }
            }
        }
        return updated_input;
    }

    public void update()
    {
        m_drive.update();
    }

    public boolean isBusy() {
        return m_drive.isBusy();
    }

    public void stop(){
        drive(0,0,0, 0, true);
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
