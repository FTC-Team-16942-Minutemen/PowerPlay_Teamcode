package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurntableSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    Servo m_turntableServo;
    double m_position;

    public static double minScale = 0.0;
    public static double maxScale = 1.0;


    public TurntableSubsystem(HardwareMap hardwareMap, Telemetry telemetry, double initial_position) {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_turntableServo = hardwareMap.get(Servo.class, "TURNTABLE");
        m_position = initial_position;
    }


    public void faceForward() {
        m_position = 1.0;
    }

    public void faceBackwards() {
        m_position = 0.0;
    }

    @Override
    public void periodic() {
        m_turntableServo.setPosition(m_position);
        m_turntableServo.scaleRange(minScale, maxScale);
//        }
//        m_telemetry.addData("Servo Pos: ", m_clawServo.getPosition());
//        m_telemetry.addData("Set Position: ", m_position);
//        m_telemetry.update();
    }
}