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
    double m_allianceHeadingOffset;
    int[] junctionX = {-48,-24,0, 24, 48};
    int[] junctionY = {-48,-24,0, 24,48};
    double REPULSERADIUS = 9.0;
    double PF_SCALE = 1.0;
    double THROTTLEMINLEVEL = 0.4;
    boolean m_isPotentialFieldEn = false;
    List<Double> wheelPositions;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialPose, double allianceHeadingOffset)
    {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
        m_drive=new SampleMecanumDrive(m_hardwareMap, m_telemetry);
        m_drive.setPoseEstimate(initialPose);
        m_allianceHeadingOffset = allianceHeadingOffset;
    }

    public void drive(double leftX, double leftY, double rightX, double throttle, boolean isFieldCentric)
    {
        Pose2d poseEstimate = getPoseEstimate();
        Vector2d input_vec = new Vector2d(leftY, leftX).rotated(
                isFieldCentric ? -(poseEstimate.getHeading() - Math.toRadians(m_allianceHeadingOffset)) :0
        );

//        Vector2d CorrectedInput = quadraticControlLaw(input_vec);
        Vector2d CorrectedInput = input_vec;

        if(m_isPotentialFieldEn)
        {
            CorrectedInput = potentialFields(input_vec);
        }

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

    public void TogglePotentialFields()
    {
        m_isPotentialFieldEn = !m_isPotentialFieldEn;
    }

    private Vector2d potentialFields(Vector2d input_vec) {
        Vector2d updated_input = input_vec;
        Pose2d poseEstimate = getPoseEstimate();

        for (int ix = 0; ix < junctionX.length; ix++) {
            for (int iy = 0; iy < junctionY.length; iy++) {
                Vector2d r_vec = new Vector2d(poseEstimate.getX(), poseEstimate.getY());
                Vector2d j_vec = new Vector2d(junctionX[ix], junctionY[iy]);

                //rotate the repulsion vector into the same frame as the input vector (not sure why the minus sign)
                Vector2d repulse_vec = r_vec.minus(j_vec).rotated(-Math.toRadians(m_allianceHeadingOffset));

                double dist = repulse_vec.norm();

                if (dist < REPULSERADIUS) {
                    Vector2d unit_repulse_vec = repulse_vec.div(dist + 0.01);
                    Vector2d new_vector = unit_repulse_vec.times(input_vec.norm() * PF_SCALE);
                    updated_input = input_vec.plus(new_vector);
//                    m_telemetry.addData("dist: ",dist);
//                    m_telemetry.addData("jx: ", junctionX[ix]);
//                    m_telemetry.addData("jy: ", junctionY[iy]);
//                    m_telemetry.addData("headingOffset: ", m_allianceHeadingOffset);
//                    m_telemetry.addData("unit_repulse_vecX: ", unit_repulse_vec.getX());
//                    m_telemetry.addData("unit_repulse_vecY: ", unit_repulse_vec.getY());
//                    m_telemetry.addData("new_vectorX: ", new_vector.getX());
//                    m_telemetry.addData("new_vectorY: ", new_vector.getY());
                }
            }
        }

//        m_telemetry.addData("updated_inputX: ",updated_input.getX());
//        m_telemetry.addData("updated_inputY: ", updated_input.getY());
//        m_telemetry.update();
        return updated_input;
    }

    public void correctHeadingOffset()
    {
        m_allianceHeadingOffset = Math.toDegrees(getPoseEstimate().getHeading());
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

    @Override
    public void periodic() {
        m_drive.updatePoseEstimate();
//        wheelPositions = m_drive.getWheelPositions();
//        m_telemetry.addData("0: ", wheelPositions.get(0));
//        m_telemetry.addData("1: ", wheelPositions.get(1));
//        m_telemetry.addData("2: ", wheelPositions.get(2));
//        m_telemetry.addData("3: ", wheelPositions.get(3));
////        m_telemetry.addData("Potential Fields Enabled: ", m_isPotentialFieldEn);
//        m_telemetry.update();
    }

}
