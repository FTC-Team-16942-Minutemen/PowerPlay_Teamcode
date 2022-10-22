package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ClawIntakeSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    Servo m_clawServo;
    double m_position;

    public static double minScale = 0.19;
    public static double maxScale = 0.55;

    public ClawIntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, double initial_position) {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
        m_clawServo = hardwareMap.get(Servo.class, "INTAKE");
        m_clawServo.scaleRange(minScale, maxScale);
        m_position = initial_position;
    }

    public void actuate()
    {
        m_position = (m_position + 1.0) % 2.0;
    }

    @Override
    public void periodic() {
        m_clawServo.scaleRange(minScale, maxScale);
        if(m_clawServo.getPosition() != m_position)
        {
            m_clawServo.setPosition(m_position);
        }
//        m_telemetry.addData("Servo Pos: ", m_clawServo.getPosition());
//        m_telemetry.addData("Set Position: ", m_position);
//        m_telemetry.update();
    }
}
