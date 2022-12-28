package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AlignmentCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ParkingCommand;
import org.firstinspires.ftc.teamcode.commands.ScoringCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.robots.triggers.LeftTriggerTrigger;
import org.firstinspires.ftc.teamcode.subsystems.ClawIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.AlignmentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurntableSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.io.IOException;

@Config
public class PowerPlayBot extends Robot {
    //Basic hardware components
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    GamepadEx m_gamePad1;
    GamepadEx m_gamePad2;

    //Subsystems
    DriveSubsystem m_driveTrain;
    LinearSlideSubsystem m_linearSlideSubsystem;
    VisionSubsystem m_visionSubsystem;
    ClawIntakeSubsystem m_clawIntakeSubsystem;
//    AlignmentSubsystem m_AlignmentSubsystem;
    TurntableSubsystem m_turntableSubsystem;

    Command m_command;
//    DistanceTrigger m_distanceTrigger;
//    TimedTrigger m_timedParkingTrigger;
    LeftTriggerTrigger m_leftTriggerTrigger;
//    Timing.Timer m_timer;
//    AutonScriptParser m_autonParser;

    public PowerPlayBot(Constants.OpModeType type,
                     HardwareMap hardwareMap,
                     Telemetry telemetry,
                     Gamepad gamePad1,
                        Gamepad gamePad2,
                        Pose2d initialPose,
                        double allianceHeadingOffset)
    {
        //Initialize basic hardware structures
        m_hardwareMap = hardwareMap;
        m_gamePad1 = new GamepadEx(gamePad1);
        m_gamePad2 = new GamepadEx(gamePad2);
        m_telemetry = telemetry;

        //Setup the FTC dashboard with it's enhanced telemetry
        m_telemetry = new MultipleTelemetry(m_telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initialize Subsystems
        m_driveTrain = new DriveSubsystem(m_hardwareMap, m_telemetry, initialPose, allianceHeadingOffset);
        m_linearSlideSubsystem = new LinearSlideSubsystem(m_hardwareMap, m_telemetry);
        m_visionSubsystem = new VisionSubsystem(m_hardwareMap, m_telemetry);
        m_clawIntakeSubsystem = new ClawIntakeSubsystem(m_hardwareMap, m_telemetry, 1.0);
        m_turntableSubsystem = new TurntableSubsystem(m_hardwareMap, m_telemetry, 0.0);
//        m_AlignmentSubsystem = new AlignmentSubsystem(m_hardwareMap, m_telemetry);

        //Setup the Robot Commands/Subsystem mappings based on OpMode type
//        m_command = new TrajectoryFollowerCommand(m_driveTrain, "TestPath");//was TestPath
//        m_distanceTrigger = new DistanceTrigger(m_DistanceSensorSubsystem, 8);
        //m_timer = new Timing.Timer(29);
        //m_timedParkingTrigger = new TimedTrigger(0.0,25.0, m_telemetry);

        m_leftTriggerTrigger = new LeftTriggerTrigger(m_gamePad1, 0.05);
//        m_autonParser = new AutonScriptParser(m_driveTrain,
//                                            m_clawIntakeSubsystem,
//                                            m_visionSubsystem,
//                                            m_linearSlideSubsystem);

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
                ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)))
                .whenInactive(new SequentialCommandGroup(
                            new WaitCommand(400),
                            new InstantCommand(() -> {m_linearSlideSubsystem.setElevatorPower(1.0);})
                        )
                );

        m_driveTrain.setDefaultCommand(new DriveCommand(m_driveTrain,
                ()->m_gamePad1.getLeftY(),
                ()->-m_gamePad1.getLeftX(),
                ()->-m_gamePad1.getRightX(),
                ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                true));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();}));

//        m_gamePad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(() -> {m_turntableSubsystem.faceForward();}));
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(() -> {m_turntableSubsystem.faceBackwards();}));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.JUNCTIONLEVEL);}));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.STACKLEVEL);}));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.GROUNDLEVEL);}));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                        new WaitCommand(400),
                        new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.ACQUIRED);})));
        m_gamePad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {m_linearSlideSubsystem.setBeaconCap();}),
                        new WaitCommand(1000),
                        new InstantCommand(() -> {m_clawIntakeSubsystem.open();})));
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.A)
//                        .whenHeld(new AlignmentCommand(m_AlignmentSubsystem, m_driveTrain,
//                                ()->m_gamePad1.getLeftY(),
//                                ()->-m_gamePad1.getLeftX(),
//                                ()->-m_gamePad1.getRightX(),
//                                ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
//                                true));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setJunctionLevel(2);}));
        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setJunctionLevel(0);}));
        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setJunctionLevel(1);}));

//        m_gamePad2.getGamepadButton(GamepadKeys.Button.BACK)
//                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.toggleOperatorMode();}));
//        m_gamePad2.getGamepadButton(GamepadKeys.Button.BACK)
//                        .whenPressed(new InstantCommand(() -> {m_driveTrain.TogglePotentialFields();}));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setStackLevel(3);}));
        m_gamePad2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setStackLevel(2);}));
        m_gamePad2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setStackLevel(1);}));
        m_gamePad2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setStackLevel(0);}));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenHeld(new InstantCommand(() -> {m_linearSlideSubsystem.lowerSlide();}))
                .whenReleased(new InstantCommand(() -> {m_linearSlideSubsystem.resetEncoder();}));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {m_driveTrain.correctHeadingOffset();}));
    }

//    private void setupTestAuton()
//    {
//        try {
//            Command autonCommand = m_autonParser.read("/sdcard/tmp/autontest.json");
//            CommandScheduler.getInstance().schedule(autonCommand);
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//
//    }


    private void setupBlueRight_Auton()
    {
//        m_timedParkingTrigger.toggleWhenActive(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}));
//        .whenInactive(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));
//                m_command.schedule();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                                new WaitCommand(300),
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.ACQUIRED);})
                        ),

                        new TrajectoryFollowerCommand(m_driveTrain, "BlueRight/BlueRight1"),
                        new TrajectoryFollowerCommand(m_driveTrain, "BlueRight/BlueRight2"),
                        new ParallelCommandGroup(
                                new TurnCommand(m_driveTrain, Math.toRadians(90)),
                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);})
                        ),
                        new TrajectoryFollowerCommand(m_driveTrain, "BlueRight/BlueRightCreep"),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.SCORING);}),
                                new WaitCommand(300),
                                new InstantCommand(() -> {m_clawIntakeSubsystem.open();}),
                                new WaitCommand(100),
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.JUNCTIONLEVEL);})
                        ),
                        new TrajectoryFollowerCommand(m_driveTrain, "BlueRight/BlueRightParking"),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.GROUNDLEVEL);}),
                                new InstantCommand(() -> {m_clawIntakeSubsystem.open();})
                        ),
                        new ParkingCommand(m_driveTrain, m_visionSubsystem,
                                "BlueRight/BlueRightParking2" ,
                                "BlueRight/BlueRightParking1",
                                "BlueRight/BlueRightParking0"
                        )
                )
        );
    }

//    private void setupBlueLeft_AutonNEW()
//    {
//        try {
//            Command autonCommand = m_autonParser.read("/sdcard/auton/LeftAuton.json");
//            CommandScheduler.getInstance().schedule(autonCommand);
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//    }


//    private void setupBlueLeft5Cone(){
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//                        new SequentialCommandGroup(new TrajectoryFollowerCommand(m_driveTrain, "RedLeft/BlueRight5Cone"));
//                )
//        );
//
//    }
//        m_timedParkingTrigger.toggleWhenActive(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}));
//        .whenInactive(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));
//                m_command.schedule();
    private void setupBlueLeft_Auton()
    {
        CommandScheduler.getInstance().schedule(
               new SequentialCommandGroup(
//


                       new SequentialCommandGroup(
                                new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                                new WaitCommand(300)
                        ),
                       new ParallelCommandGroup(
                               new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/Left1"),
                               new SequentialCommandGroup(
                                       new WaitCommand(1000),
                                       new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);}))
                       ),
                       new SequentialCommandGroup(
                               new InstantCommand(() -> {m_clawIntakeSubsystem.open();}),
                               new WaitCommand(300)
                       ),
                       new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/Left2"),
                       new WaitCommand(50),
                       new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.STACKLEVEL, 3);}),
                       new TurnCommand(m_driveTrain, Math.toRadians(-178)),
                       new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/Left3"),
                       new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                       new WaitCommand(300),
                       new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 0);}),
                       new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/ConeBack"),
                       new TurnCommand(m_driveTrain, Math.toRadians(178)),
                       new ParallelCommandGroup(
                                    new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/Cycler"),
                                    new SequentialCommandGroup(
                                            new WaitCommand(500),
                                            new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);})
                                                     )
                                        ),
                      new InstantCommand(() -> {m_clawIntakeSubsystem.open();}),
                                    //repeat starts here
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/ConeGetter"),
                                                new SequentialCommandGroup(
                                                    new WaitCommand(500),
                                                    new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.STACKLEVEL, 2);})
                                                    )
                                                ),
                                        new TurnCommand(m_driveTrain, Math.toRadians(180)),
                                        new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/ConeAquire"),

                                        new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 0);}),
                                        new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/ConeBack"),
                                        new TurnCommand(m_driveTrain, Math.toRadians(180)),

                                        new ParallelCommandGroup(
                                                new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/Cycler"),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(500),
                                                        new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);}))),
                                        new InstantCommand(() -> {m_clawIntakeSubsystem.open();})),

                                //new cycle
                                new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                        new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/ConeGetter"),
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(500),
                                                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.STACKLEVEL, 1);})
                                                        )
                                                ),
                                                new TurnCommand(m_driveTrain, Math.toRadians(180)),
                                                new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/ConeAquire"),

                                                new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                                                new WaitCommand(300),
                                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 0);}),
                                                new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/ConeBack"),
                                                new TurnCommand(m_driveTrain, Math.toRadians(180)),

                                                new ParallelCommandGroup(
                                                        new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/Cycler"),
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(500),
                                                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);}))
                                                ),
                                                new InstantCommand(() -> {m_clawIntakeSubsystem.open();}))
//        // new cycle
//        new SequentialCommandGroup(
//                new ParallelCommandGroup(
//                        new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/ConeGetter"),
//                        new SequentialCommandGroup(
//                                new WaitCommand(500),
//                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.STACKLEVEL, 0);})
//                        )
//                ),
//                new TurnCommand(m_driveTrain, Math.toRadians(180)),
//                new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/ConeAquire"),
//
//                new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
//                new WaitCommand(300),
//                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 0);}),
//                new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/ConeBack"),
//                new TurnCommand(m_driveTrain, Math.toRadians(180)),
//
//                new ParallelCommandGroup(
//                        new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/Cycler"),
//                        new SequentialCommandGroup(
//                                new WaitCommand(500),
//                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);}))
//                ),
//                new InstantCommand(() -> {m_clawIntakeSubsystem.open();}))

               ));

///
    }
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
//                                new WaitCommand(300)
//                        ),
////
//
//                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeft/BlueLeft1"),
//                        new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);}),
//                        new InstantCommand(() -> {m_clawIntakeSubsystem.open();})
//                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeft/BlueLeft2"),
//                        new ParallelCommandGroup(
//                            //    new TurnCommand(m_driveTrain, Math.toRadians(-90.0)),
//                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);})
//                        ),
//                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeft/BlueLeftCreep"),
//                        new WaitCommand(800),
//                        new SequentialCommandGroup(
////                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.SCORING);}),
////                                new WaitCommand(300),
//                                new InstantCommand(() -> {m_clawIntakeSubsystem.open();}),
//                                new WaitCommand(300)
////                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.JUNCTIONLEVEL);})
//                        ),
//                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeft/BlueLeftPark2"),
//
//////START OF NEW
//                        new ParallelCommandGroup(
//                                new TurnCommand(m_driveTrain, Math.toRadians(180.0)),
//                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.STACKLEVEL, 3);})
//                        ),
//                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeft/BlueLeftStack1"),
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
//                                new WaitCommand(300),
//                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.ACQUIRED);})
//                        ),
//                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeft/BlueLeftStack2"),
//                        new ParallelCommandGroup(
//                                new TurnCommand(m_driveTrain, Math.toRadians(-90.0)),
//                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);})
//                        ),
//                        new TrajectoryFollowerCommand(m_driveTrain,"BlueLeft/BlueLeftCreep2"),
//                        new SequentialCommandGroup(
////                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.SCORING);}),
////                                new WaitCommand(300),
//                                new InstantCommand(() -> {m_clawIntakeSubsystem.open();}),
//                                new WaitCommand(300)
////                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.JUNCTIONLEVEL);})
//                        ),
//                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeft/ReverseBlueLeftCreep"),
//                        new TurnCommand(m_driveTrain, Math.toRadians(90.0)),
//
//                        new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.STACKLEVEL, 2);}),
//
//                        new TrajectoryFollowerCommand(m_driveTrain, "BlueLeft/BlueLeftStack5"),
//
//                        new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
//                        new WaitCommand(300),
//                        new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.ACQUIRED);}),
//                        new WaitCommand(800),
//
//                        new ParkingCommand(m_driveTrain, m_visionSubsystem,
//                                "BlueLeft/BlueLeftStackParking0",
//                                "BlueLeft/BlueLeftStackParking1",
//                                "BlueLeft/BlueLeftStackParking2"
//                        ),
//                        new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.GROUNDLEVEL);})

//                )
//        );

    private void setupRedRight_Auton()
    {
//
//        m_timedParkingTrigger.toggleWhenActive(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}));
//        .whenInactive(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));
//                m_command.schedule();
                CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                                new WaitCommand(300)
                        ),
//

                        new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/Left1"),
                        new TrajectoryFollowerCommand(m_driveTrain, "LeftAuton/Left2"),
                        new ParallelCommandGroup(
                                new TurnCommand(m_driveTrain, Math.toRadians(90.0)),
                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);})
                        ),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedRight/RedRightCreep"),
                        new WaitCommand(800),
                        new SequentialCommandGroup(
       //                         new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.SCORING);}),
//                                new WaitCommand(300),
                                new InstantCommand(() -> {m_clawIntakeSubsystem.open();}),
                                new WaitCommand(300)
//                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.JUNCTIONLEVEL);})
                        ),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedRight/RedRightPark2"),
//
////START OF NEW
                        new ParallelCommandGroup(
                                new TurnCommand(m_driveTrain, Math.toRadians(-180.0)),
                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.STACKLEVEL, 3);})
                        ),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedRight/RedRightStack1"),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                                new WaitCommand(500),
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.ACQUIRED);})
                        ),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedRight/RedRightStack2"),
                        new ParallelCommandGroup(
                                new TurnCommand(m_driveTrain, Math.toRadians(90.0)),
                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);})
                        ),
                        new TrajectoryFollowerCommand(m_driveTrain,"RedRight/RedRightCreep2"),
                        new WaitCommand(300),
                        new SequentialCommandGroup(
       //                         new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.SCORING);}),
//                                new WaitCommand(300),
                                new InstantCommand(() -> {m_clawIntakeSubsystem.open();}),
                                new WaitCommand(300)
     //                           new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.JUNCTIONLEVEL);})
                        ),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedRight/ReverseCreep"),
                        new TurnCommand(m_driveTrain, Math.toRadians(-90.0)),
//
                        new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.STACKLEVEL, 2);}),

                        new TrajectoryFollowerCommand(m_driveTrain, "RedRight/RedRightStack5"),

                        new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                        new WaitCommand(500),
                        new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.ACQUIRED);}),
                        new WaitCommand(800),
//
                        new ParkingCommand(m_driveTrain, m_visionSubsystem,
                                "RedRight/RedRightStackParking2",
                                "RedRight/RedRightStackParking1",
                                "RedRight/RedRightStackParking0"
                        ),
                        new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.GROUNDLEVEL);})
//                        )
                )
        );
    }
    private void setupRedLeft_Auton()
    {
//        m_timedParkingTrigger.toggleWhenActive(new InstantCommand(() -> {m_linearSlideSubsystem.step(1);}));
//        .whenInactive(new InstantCommand(() -> {m_linearSlideSubsystem.step(-1);}));
//                m_command.schedule();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                                new WaitCommand(300),
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.ACQUIRED);})
                        ),                        new TrajectoryFollowerCommand(m_driveTrain, "RedLeft/RedLeft1"),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedLeft/RedLeft2"),
                        new ParallelCommandGroup(
                                new TurnCommand(m_driveTrain, Math.toRadians(-90)),
                                new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, 2);})
                        ),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedLeft/RedLeftCreep"),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.SCORING);}),
                                new WaitCommand(300),
                                new InstantCommand(() -> {m_clawIntakeSubsystem.open();}),
                                new WaitCommand(100),
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.JUNCTIONLEVEL);})
                        ),
                        new TrajectoryFollowerCommand(m_driveTrain, "RedLeft/RedLeftPark"),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.GROUNDLEVEL);}),
                                new InstantCommand(() -> {m_clawIntakeSubsystem.open();})
                        ),
                        new ParkingCommand(m_driveTrain,m_visionSubsystem ,
                                "RedLeft/RedLeftParking0",
                                "RedLeft/RedLeftParking1",
                                "RedLeft/RedLeftParking2"
                        )
                )
        );
    }



}

