package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.robots.Constants.LinearSlideState;

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
import org.firstinspires.ftc.teamcode.robots.Constants.OperatorMode;

@Config
public class LinearSlideSubsystem extends SubsystemBase {
    public static final int MAXCONECOUNT = 4;
    public static final int MAXJUNCTIONCOUNT = 3;

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    DcMotorEx m_LinearSlideMotor;

    //Used for 2500 Velocity Setpoint
    public static double p = 9.0;
    public static double i = 0.0;
    public static double d = 1.0;
    public static double f = 14.0;

    public static double position_p = 8.5;
    public static double downElevatorPower = 1.0;
    public static double upElevatorPower = 1.0;

    public static int junctionHigh = 2750; //was 3150
    public static int junctionMed = 2034;
    public static int junctionLow = 1200;
    public static int junctionGnd = 300;

    public static int cone1Pos = 60;
    public static int cone2Pos = 130;
    public static int cone3Pos = 260;
    public static int cone4Pos = 360;

    public static int acquiredOffset = 500;
    public static int scoringOffset = 200;

    private int m_position_index = 0;
    private int m_stackPosition_index = 0;
    private int m_error = 0;

    private int m_targetPosition = 0;
    private double m_targetPower = 0.0;
    private LinearSlideState m_currentState;
    private int m_junctionIndex = 0;
    private int m_stackIndex = 0;
    private OperatorMode m_operatorMode = OperatorMode.DOUBLE_OPERATOR_MODE;

    int[] junctionPositions = { junctionLow, junctionMed, junctionHigh};
    int[] stackPositions = {cone1Pos, cone2Pos, cone3Pos, cone4Pos};
    //high junction: 2900   medium junction: 1500   small Junction: 750  ground:

    public LinearSlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
        m_LinearSlideMotor = hardwareMap.get(DcMotorEx.class, "LS");
        m_LinearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        m_LinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_LinearSlideMotor.setTargetPosition(m_targetPosition);
        m_currentState = LinearSlideState.GROUNDLEVEL;
        m_LinearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        m_LinearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stateTransition(LinearSlideState requestedState)
    {
        switch(requestedState)
        {
            case JUNCTIONLEVEL:
                if((m_currentState == requestedState) & (m_operatorMode == OperatorMode.SINGLE_OPERATOR_MODE)) // only passing the junction level setting to the 2nd controller
                {
                    m_junctionIndex = (m_junctionIndex + 1) % junctionPositions.length;
                }
                m_targetPosition = junctionPositions[m_junctionIndex];
                m_currentState = requestedState;
                break;
            case SCORING:
                if(m_currentState == requestedState)
                {
                }
                else
                {
                    m_targetPosition = Math.max(m_LinearSlideMotor.getTargetPosition() - scoringOffset, 0);
                }
                m_currentState = requestedState;
                break;
            case ACQUIRED:
                if(m_currentState == requestedState)
                {
//                    m_telemetry.addData("state: ", requestedState);
                }
                else
                {
                    m_targetPosition = Math.min(m_LinearSlideMotor.getTargetPosition() + acquiredOffset, junctionPositions[junctionPositions.length-1]);
                }
//                m_telemetry.addData("m_acquiredIndex: ", m_acquiredIndex);
//                m_telemetry.update();

                m_currentState = requestedState;

                break;
            case STACKLEVEL:
                if((m_currentState == requestedState) & (m_operatorMode == OperatorMode.SINGLE_OPERATOR_MODE)) // only passing the junction level setting to the 2nd controller
                {
                    m_stackIndex = (m_stackIndex + 1) % stackPositions.length;
//                    m_telemetry.addData("state: ", requestedState);
                }
//                m_telemetry.addData("m_stackIndex: ", m_stackIndex);
//                m_telemetry.update();
                m_targetPosition = stackPositions[m_stackIndex];
                m_currentState = requestedState;

                break;
            case GROUNDLEVEL:
                m_targetPosition = 0;
                m_currentState = requestedState;
                break;
        }

        m_targetPower = calcDefaultPower();
    }

    private double calcDefaultPower()
    {
        m_error = m_targetPosition - m_LinearSlideMotor.getCurrentPosition();

        if(m_error >= 0)
        {
            return upElevatorPower;
        }
        else
        {
            return downElevatorPower;
        }
    }

    public void setState(LinearSlideState requestedState, int desiredIndex)
    {
        switch(requestedState)
        {
            case JUNCTIONLEVEL:
                m_targetPosition = junctionPositions[desiredIndex];
                m_junctionIndex = desiredIndex;
                break;
            case SCORING:
                break;
            case ACQUIRED:
                break;
            case STACKLEVEL:
                m_targetPosition = stackPositions[desiredIndex];
                m_stackIndex = desiredIndex;
                break;
            case GROUNDLEVEL:
                m_targetPosition = -25;
                break;
        }
        m_currentState = requestedState;
        m_targetPower = calcDefaultPower();
    }

    public void setJunctionLevel(int desiredLevelIndex)
    {
        if(m_operatorMode == OperatorMode.DOUBLE_OPERATOR_MODE)
        {
            m_junctionIndex = desiredLevelIndex;
            m_telemetry.addData("JUNCTION index: ", m_junctionIndex);
            m_telemetry.update();
        }
    }

    public void setStackLevel(int desiredLevelIndex)
    {
        if(m_operatorMode == OperatorMode.DOUBLE_OPERATOR_MODE)
        {
            m_stackIndex = desiredLevelIndex;
            m_telemetry.addData("STACK index: ", m_stackIndex);
            m_telemetry.update();
        }
    }

    public void setBeaconCap()
    {
        m_targetPosition = Math.max(m_targetPosition - scoringOffset - scoringOffset, 0);
        m_targetPower = calcDefaultPower() * 0.5;
    }

    public void toggleOperatorMode()
    {
        m_operatorMode = (m_operatorMode == OperatorMode.SINGLE_OPERATOR_MODE) ? OperatorMode.DOUBLE_OPERATOR_MODE : OperatorMode.SINGLE_OPERATOR_MODE;
    }

    public boolean isLinearSlideBusy()
    {
        return m_LinearSlideMotor.isBusy();
    }

    public int getPositionSetpoint() {
        return m_LinearSlideMotor.getTargetPosition();
    }

    public void setPositionSetPoint(int positionSetpoint) {
        m_targetPosition = positionSetpoint;
    }

    public void setElevatorPower(double power)
    {
        m_targetPower = power;
    }

    public double getElevatorPower()
    {
        return m_LinearSlideMotor.getPower();
    }

    public double getElevatorPosition() {return m_LinearSlideMotor.getCurrentPosition();}

    public void lowerSlide() {
        m_LinearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_targetPower = -0.4;
    }

    public void resetEncoder(){
        m_targetPower = 0.0;
        m_targetPosition = 0;
        m_LinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_LinearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_currentState = LinearSlideState.GROUNDLEVEL;
    }

    @Override
    public void periodic() {
        junctionPositions[0] = junctionLow;
        junctionPositions[1] = junctionMed;
        junctionPositions[2] = junctionHigh;

        stackPositions[0] = cone1Pos;
        stackPositions[1] = cone2Pos;
        stackPositions[2] = cone3Pos;
        stackPositions[3] = cone4Pos;
        m_telemetry.addData("TruePosition: ", m_LinearSlideMotor.getCurrentPosition());
        m_telemetry.addData("DesiredPosition: ", m_targetPosition);
        m_telemetry.addData("DesiredPower: ", m_targetPower);
        m_telemetry.update();
        m_LinearSlideMotor.setVelocityPIDFCoefficients(p, i, d, f);
        m_LinearSlideMotor.setPositionPIDFCoefficients(position_p);

        m_LinearSlideMotor.setTargetPosition(m_targetPosition);
        m_LinearSlideMotor.setPower(m_targetPower);
    }


}
