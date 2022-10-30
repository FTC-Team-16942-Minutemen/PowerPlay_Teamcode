package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
public class LinearSlideSubsystem extends SubsystemBase {
    public static int desiredPosition = 0;
    public static int positionSetpoint = 400;
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    DcMotorEx m_LinearSlideMotor;

    //Used for 2500 Velocity Setpoint
    public static double p = 9.0;
    public static double i = 0.0;
    public static double d = 1.0;
    public static double f = 14.0;

    public static double position_p = 8.0;
    public static double downElevatorPower = 0.6;
    public static double upElevatorPower = 1.0;

    public static int junctionHigh = 2850; //was 3150
    public static int junctionMed = 2034;
    public static int junctionLow = 1300;
    public static int junctionGnd = 300;
    public static int lowestPoint = 0;

    private int m_position_index = 0;
    private int m_error = 0;
    int[] positions = {lowestPoint, junctionGnd, junctionLow, junctionMed, junctionHigh};
    //high junction: 2900   medium junction: 1500   small Junction: 750  ground:

    public LinearSlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
        m_LinearSlideMotor = hardwareMap.get(DcMotorEx.class, "LS");
        m_LinearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        m_LinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_LinearSlideMotor.setTargetPosition(0);

        m_LinearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        m_LinearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void step(int direction) {
        m_position_index = m_position_index + direction;
        if (m_position_index >= positions.length) {
            m_position_index = positions.length - 1;
        }
        if (m_position_index < 0) {
            m_position_index = 0;
        }

        desiredPosition = positions[m_position_index];

        m_LinearSlideMotor.setTargetPosition(desiredPosition);

        m_error = desiredPosition - m_LinearSlideMotor.getCurrentPosition();

        if(m_error >= 0)
        {
            m_LinearSlideMotor.setPower(upElevatorPower);
        }
        else
        {
            m_LinearSlideMotor.setPower(downElevatorPower);
        }
    }

    public void extend(double input) {
        m_LinearSlideMotor.setPower(input);
    }
    public void stop() {

    }

    public boolean isLinearSlideBusy()
    {
        return m_LinearSlideMotor.isBusy();
    }

    public int getPositionSetpoint() {
        return m_LinearSlideMotor.getTargetPosition();
    }

    public void setPositionSetPoint(int positionSetpoint) {
     m_LinearSlideMotor.setTargetPosition(positionSetpoint);
    }

    public void setElevatorPower(double power)
    {
        m_LinearSlideMotor.setPower(power);
    }

    public double getElevatorPower()
    {
        return m_LinearSlideMotor.getPower();
    }

    @Override
    public void periodic() {
        positions[0] = lowestPoint;
        positions[1] = junctionGnd;
        positions[2] = junctionLow;
        positions[3] = junctionMed;
        positions[4] = junctionHigh;
       // m_telemetry.addData("TruePosition", m_LinearSlideMotor.getCurrentPosition());
        //m_telemetry.addData("DesiredPosition", desiredPosition);
        //m_telemetry.update();
        m_LinearSlideMotor.setVelocityPIDFCoefficients(p, i, d, f);
        m_LinearSlideMotor.setPositionPIDFCoefficients(position_p);
//
//        m_LinearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


}
