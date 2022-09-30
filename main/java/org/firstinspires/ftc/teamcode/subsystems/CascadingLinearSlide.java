package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.DecompositionSolver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class CascadingLinearSlide extends SubsystemBase {

    public static double DesiredVelocity2 = 500;
    //public static double ActualVelocity2 = 0;
    //public static double RealPosition;
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    DcMotorEx m_CascadingLinearSlideMotor;

    //Used for a velocity of 2500
    public static double p = 9.0;
    public static double i = 0.0;
    public static double d = 1.0;
    public static double f = 14.0;


    private double velocityConstant;
    private int position_index;




    public CascadingLinearSlide(HardwareMap hardwareMap, Telemetry telemetry) {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
        m_CascadingLinearSlideMotor = hardwareMap.get(DcMotorEx.class, "CLS");
        m_CascadingLinearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        m_CascadingLinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //m_CascadingLinearSlideMotor.setTargetPosition(0);
        //m_CascadingLinearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void up() {
        m_CascadingLinearSlideMotor.setVelocity(DesiredVelocity2);
        DesiredVelocity2 = 2500;

    }
    public void down() {
        m_CascadingLinearSlideMotor.setVelocity(-DesiredVelocity2);
        DesiredVelocity2 = -2500;
    }

    public void stop() {
        m_CascadingLinearSlideMotor.setVelocity(0);
        DesiredVelocity2 = 0;
    }
    @Override
    public void periodic() {
        DesiredVelocity2 = DesiredVelocity2 + velocityConstant;
        //m_telemetry.addData("TrueVelocity", m_CascadingLinearSlideMotor.getVelocity());
        //m_telemetry.addData("DesiredVelocity", DesiredVelocity2);
        //m_telemetry.update();
        m_CascadingLinearSlideMotor.setVelocityPIDFCoefficients(p, i, d, f);
        //m_CascadingLinearSlideMotor.setVelocityPIDFCoefficients(p, i, d, f);
        //m_CascadingLinearSlideMotor.setPositionPIDFCoefficients(position_p);
//
        //m_CascadingLinearSlideMotor.setTargetPosition(position_index);
        //m_CascadingLinearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //m_CascadingLinearSlideMotor.setVelocity(DesiredVelocity2);
    }


}
