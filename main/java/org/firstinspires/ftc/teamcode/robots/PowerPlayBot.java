package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoTargetingDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.robots.triggers.DistanceTrigger;
import org.firstinspires.ftc.teamcode.subsystems.CascadingLinearSlide;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Config
public class PowerPlayBot extends Robot {
    //Basic hardware components
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    GamepadEx m_gamePad1;

    //Subsystems
    //DriveSubsystem m_driveTrain;
    //LinearSlideSubsystem m_linearSlideSubsystem;
    VisionSubsystem m_visionSubsystem;
    //CascadingLinearSlide m_CascadingLinearSlide;
    //DistanceSensorSubsystem m_DistanceSensorSubsystem;

    Command m_command;
    DistanceTrigger m_distanceTrigger;

    public PowerPlayBot(Constants.OpModeType type,
                     HardwareMap hardwareMap,
                     Telemetry telemetry,
                     Gamepad gamePad1)
    {
        //Initialize basic hardware structures
        m_hardwareMap = hardwareMap;
        m_gamePad1 = new GamepadEx(gamePad1);
        m_telemetry = telemetry;

        //Setup the FTC dashboard with it's enhanced telemetry
        m_telemetry = new MultipleTelemetry(m_telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initialize Subsystems
//        m_driveTrain = new DriveSubsystem(m_hardwareMap, m_telemetry);
//        m_linearSlideSubsystem = new LinearSlideSubsystem(m_hardwareMap, m_telemetry);
        m_visionSubsystem = new VisionSubsystem(m_hardwareMap, m_telemetry);
//        m_CascadingLinearSlide = new CascadingLinearSlide(m_hardwareMap, m_telemetry);
//        m_DistanceSensorSubsystem = new DistanceSensorSubsystem(m_hardwareMap, m_telemetry);

        //Setup the Robot Commands/Subsystem mappings based on OpMode type
//        m_command = new TrajectoryFollowerCommand(m_driveTrain, "TestPath");//was TestPath
//        m_distanceTrigger = new DistanceTrigger(m_DistanceSensorSubsystem, 8);

        setupOpMode(type);
    }

    private void setupOpMode(Constants.OpModeType type)
    {
        if(type == Constants.OpModeType.TELEOP)
        {
            m_telemetry.addData("Initialize","TeleOp");
            setupTeleOp();
        }
        else if (type == Constants.OpModeType.AUTO)
        {
            m_telemetry.addData("Initialize", "Auton");
            setupAuton();
        }

        m_telemetry.update();
    }

    private void setupTeleOp()
    {
       //m_distanceTrigger.whenActive(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}))
               //.whenInactive(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
//                .whenHeld(new InstantCommand(() -> {m_DistanceSensorSubsystem.getDistance();}));
//
//        m_driveTrain.setDefaultCommand(new DriveCommand(m_driveTrain,
//                ()->m_gamePad1.getLeftY(),
//                ()->-m_gamePad1.getLeftX(),
//                ()->-m_gamePad1.getRightX(),
//                true));
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}));
//
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));
//
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.Y)
//                .whenHeld(new InstantCommand(() -> {m_CascadingLinearSlide.up();}));
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new InstantCommand(() -> {m_CascadingLinearSlide.stop();}));
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.X)
//                .whenHeld(new InstantCommand(() -> {m_CascadingLinearSlide.down();}));
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenHeld(new AutoTargetingDriveCommand(m_driveTrain,
//                        m_visionSubsystem,
//                        ()->m_gamePad1.getLeftY(),
//                        ()->-m_gamePad1.getLeftX(),
//                        ()->-m_gamePad1.getRightX(),
//                        true));
    }


    private void setupAuton()
    {//        m_command.schedule();
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//                        new TrajectoryFollowerCommand(m_driveTrain, "TestPath"),
//                        new TrajectoryFollowerCommand(m_driveTrain, "TestPath2"),
//                        new TrajectoryFollowerCommand(m_driveTrain, "TestPath3"),
//                        new TrajectoryFollowerCommand(m_driveTrain, "Testing Brother John"),
//                        new TrajectoryFollowerCommand(m_driveTrain,"PoleRun"),
//                        new TurnCommand(m_driveTrain, 3.1415926)
//                ));
    }

}

