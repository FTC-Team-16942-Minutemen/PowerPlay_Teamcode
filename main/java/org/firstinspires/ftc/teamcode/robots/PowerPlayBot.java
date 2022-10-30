package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoTargetingDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ParkingCommand;
import org.firstinspires.ftc.teamcode.commands.ScoringCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.robots.triggers.DistanceTrigger;
import org.firstinspires.ftc.teamcode.robots.triggers.LeftTriggerTrigger;
import org.firstinspires.ftc.teamcode.robots.triggers.TimedTrigger;
import org.firstinspires.ftc.teamcode.subsystems.ClawIntakeSubsystem;
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
    DriveSubsystem m_driveTrain;
    LinearSlideSubsystem m_linearSlideSubsystem;
    VisionSubsystem m_visionSubsystem;
    ClawIntakeSubsystem m_clawIntakeSubsystem;
    //CascadingLinearSlide m_CascadingLinearSlide;
    //DistanceSensorSubsystem m_DistanceSensorSubsystem;

    Command m_command;
//    DistanceTrigger m_distanceTrigger;
//    TimedTrigger m_timedParkingTrigger;
    LeftTriggerTrigger m_leftTriggerTrigger;
//    Timing.Timer m_timer;

    public PowerPlayBot(Constants.OpModeType type,
                     HardwareMap hardwareMap,
                     Telemetry telemetry,
                     Gamepad gamePad1,
                        Pose2d initialPose,
                        double allianceHeadingOffset)
    {
        //Initialize basic hardware structures
        m_hardwareMap = hardwareMap;
        m_gamePad1 = new GamepadEx(gamePad1);
        m_telemetry = telemetry;

        //Setup the FTC dashboard with it's enhanced telemetry
        m_telemetry = new MultipleTelemetry(m_telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initialize Subsystems
        m_driveTrain = new DriveSubsystem(m_hardwareMap, m_telemetry, initialPose, allianceHeadingOffset);
        m_linearSlideSubsystem = new LinearSlideSubsystem(m_hardwareMap, m_telemetry);
        m_visionSubsystem = new VisionSubsystem(m_hardwareMap, m_telemetry);
        m_clawIntakeSubsystem = new ClawIntakeSubsystem(m_hardwareMap, m_telemetry, 1.0);
//        m_CascadingLinearSlide = new CascadingLinearSlide(m_hardwareMap, m_telemetry);
//        m_DistanceSensorSubsystem = new DistanceSensorSubsystem(m_hardwareMap, m_telemetry);

        //Setup the Robot Commands/Subsystem mappings based on OpMode type
//        m_command = new TrajectoryFollowerCommand(m_driveTrain, "TestPath");//was TestPath
//        m_distanceTrigger = new DistanceTrigger(m_DistanceSensorSubsystem, 8);
        //m_timer = new Timing.Timer(29);
        //m_timedParkingTrigger = new TimedTrigger(0.0,25.0, m_telemetry);
        m_leftTriggerTrigger = new LeftTriggerTrigger(m_gamePad1, 0.05);

        setupOpMode(type);
    }

    public void disableVision()
    {
        m_visionSubsystem.disablePipeline();
    }

//    public void setCurrentTime(double time)
//    {
//        m_timedParkingTrigger.setTime(time);
//    }

    public Pose2d getRobotPose()
    {
        return m_driveTrain.getPoseEstimate();
    }

//    public void setRobotPose(Pose2d inputPose)
//    {
//        m_driveTrain.setPoseEstimate(inputPose);
//    }

    private void setupOpMode(Constants.OpModeType type)
    {
        if(type == Constants.OpModeType.TELEOP)
        {
            m_telemetry.addData("Initialize","TeleOp");
            setupTeleOp();
        }
        else if (type == Constants.OpModeType.BLUE_RIGHT_AUTO)
        {
            m_telemetry.addData("Initialize", "BlueRight_Auton");
            setupBlueRight_Auton();
        }
        else if (type == Constants.OpModeType.BLUE_LEFT_AUTO)
        {
            m_telemetry.addData("Initialize", "BlueLeft_Auton");
            setupBlueLeft_Auton();
        }
        else if (type == Constants.OpModeType.RED_RIGHT_AUTO)
        {
            m_telemetry.addData("Initialize", "RedRight_Auton");
            setupRedRight_Auton();
        }
        else if (type == Constants.OpModeType.RED_LEFT_AUTO)
        {
            m_telemetry.addData("Initialize", "RedLeft_Auton");
            setupRedLeft_Auton();
        }

        m_telemetry.update();
    }

    private void setupTeleOp()
    {
        m_leftTriggerTrigger.whileActiveOnce(new ScoringCommand(m_clawIntakeSubsystem,
                m_linearSlideSubsystem,
                ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

        //m_distanceTrigger.whenActive(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}))
               //.whenInactive(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
//                .whenHeld(new InstantCommand(() -> {m_DistanceSensorSubsystem.getDistance();}));
//
//        m_driveTrain.setPoseEstimate(new Pose2d(new Vector2d( 35.0, 60.0), -3.145926/2.0));
//        m_gamePad1.
                //                .whenHeld(new ScoringCommand(m_clawIntakeSubsystem,
//                        m_linearSlideSubsystem,
//                        ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
//                        ));
        m_linearSlideSubsystem.setDefaultCommand(new InstantCommand(() -> {m_linearSlideSubsystem.step(0);}));

        m_driveTrain.setDefaultCommand(new DriveCommand(m_driveTrain,
                ()->m_gamePad1.getLeftY(),
                ()->-m_gamePad1.getLeftX(),
                ()->-m_gamePad1.getRightX(),
                ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                true));
        m_gamePad1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}));
        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();}));
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenHeld(new InstantCommand(() -> {m_linearSlideSubsystem.extend(1);}))
//                .whenReleased(new InstantCommand(()->{m_linearSlideSubsystem.extend(0);}));
//
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenHeld(new InstantCommand(() -> {m_linearSlideSubsystem.extend(-1);}))
//                .whenReleased(new InstantCommand(()->{m_linearSlideSubsystem.extend(0);}));
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


    private void setupBlueRight_Auton()
    {
//        m_timedParkingTrigger.toggleWhenActive(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}));
//        .whenInactive(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));
//                m_command.schedule();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                         new TrajectoryFollowerCommand(m_driveTrain, "BlueRight1"),
                        new TrajectoryFollowerCommand(m_driveTrain, "BlueRight2"),
                        new ParallelCommandGroup(
                                new TurnCommand(m_driveTrain, Math.toRadians(90)),
                                new InstantCommand(() -> {m_linearSlideSubsystem.step(4);})),
                        new TrajectoryFollowerCommand(m_driveTrain, "BlueRightCreep"),
                        new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();}),
                        new TrajectoryFollowerCommand(m_driveTrain, "BlueRightParking"),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> {m_linearSlideSubsystem.step(-4);}),
                                new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();})
                        ),
                        new ParkingCommand(m_driveTrain,m_visionSubsystem ,
                                "BlueRightParking0" ,
                                "BlueRightParking1",
                                "BlueRightParking2"
                        )

                ));
    }
    private void setupBlueLeft_Auton()
    {
//        m_timedParkingTrigger.toggleWhenActive(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}));
//        .whenInactive(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));
//                m_command.schedule();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeft1"),
                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeft2"),
                        new ParallelCommandGroup(
                                new TurnCommand(m_driveTrain, Math.toRadians(-90)),
                                new InstantCommand(() -> {m_linearSlideSubsystem.step(4);})),
                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeftCreep"),
                        new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();}),
                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeftPark2"),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> {m_linearSlideSubsystem.step(-4);}),
                                new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();})
                        ),
                        new ParkingCommand(m_driveTrain,m_visionSubsystem ,
                                "BlueLeftParking0" ,
                                "BlueLeftParking1",
                                "BlueLeftParking2"
                                )

                ));
    }
    private void setupRedRight_Auton()
    {
//        m_timedParkingTrigger.toggleWhenActive(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}));
//        .whenInactive(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));
//                m_command.schedule();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedRight1"),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedRight2"),
                        new ParallelCommandGroup(
                                new TurnCommand(m_driveTrain, Math.toRadians(90)),
                                new InstantCommand(() -> {m_linearSlideSubsystem.step(4);})),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedRightCreep"),
                        new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();}),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedRightPark"),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> {m_linearSlideSubsystem.step(-4);}),
                                new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();})
                        ),
                        new ParkingCommand(m_driveTrain,m_visionSubsystem ,
                                "RedRightParking0" ,
                                "RedRightParking1",
                                "RedRightParking2"
                        )

                ));
    }
    private void setupRedLeft_Auton()
    {
//        m_timedParkingTrigger.toggleWhenActive(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}));
//        .whenInactive(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));
//                m_command.schedule();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedLeft1"),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedLeft2"),
                        new ParallelCommandGroup(
                                new TurnCommand(m_driveTrain, Math.toRadians(-90)),
                                new InstantCommand(() -> {m_linearSlideSubsystem.step(4);})),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedLeftCreep"),
                        new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();}),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedLeftPark"),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> {m_linearSlideSubsystem.step(-4);}),
                                new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();})
                        ),
                        new ParkingCommand(m_driveTrain,m_visionSubsystem ,
                                "RedLeftParking0" ,
                                "RedLeftParking1",
                                "RedLeftParking2"
                        )

                ));
    }



}

