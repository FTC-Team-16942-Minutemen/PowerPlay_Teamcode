package org.firstinspires.ftc.teamcode.robots;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ParkingCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.FileReader;
import java.io.IOException;
import java.util.Iterator;
import java.util.Locale;

public class AutonScriptParser {


    JSONParser parser = new JSONParser();
    DriveSubsystem m_driveSubsystem;
    ClawIntakeSubsystem m_clawSubsystem;
    VisionSubsystem m_visionSubsystem;
    LinearSlideSubsystem m_linearSlideSubsystem;


    public AutonScriptParser(DriveSubsystem driveSubsystem,
                             ClawIntakeSubsystem clawSubsystem,
                             VisionSubsystem visionSubsystem,
                             LinearSlideSubsystem linearSlideSubsystem)
    {
        m_driveSubsystem = driveSubsystem;
        m_clawSubsystem = clawSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_linearSlideSubsystem = linearSlideSubsystem;
    }

    @SuppressLint("NewApi")
    public CommandGroupBase read(String inputJsonString) throws IOException {
        Object obj = null;

        try
        {
            obj = parser.parse(new FileReader(inputJsonString));
        }
        catch (ParseException e) {
            e.printStackTrace();
        }

        JSONObject jsonObject = (JSONObject)obj;
        JSONArray inputCommandArray = (JSONArray)jsonObject.get("SeqCommandGroup");

        CommandGroupBase outCommand = parseJsonCommands(new SequentialCommandGroup(), inputCommandArray);
        return outCommand;
//        return commandGroup;
    }

    public CommandGroupBase parseJsonCommands(CommandGroupBase inputCommandGroup, JSONArray jsonCommandList)
    {
        for(Iterator i = jsonCommandList.iterator(); i.hasNext();)
        {
            JSONObject object = (JSONObject)i.next();
            Object key = object.keySet().iterator().next();
            Object subObject = object.get(key);

            if(subObject instanceof JSONArray)
            {
                CommandGroupBase newCommandGroup;
                switch (key.toString())
                {
                    case "SeqCommandGroup":
                        newCommandGroup = parseJsonCommands(new SequentialCommandGroup(), (JSONArray)subObject);
                        inputCommandGroup.addCommands(newCommandGroup);
                        break;
                    case "ParallelCommandGroup":
                        newCommandGroup = parseJsonCommands(new ParallelCommandGroup(), (JSONArray)subObject);
                        inputCommandGroup.addCommands(newCommandGroup);
                        break;
                }

            }
            else
            {
                String parameter = subObject.toString();
                String[] params;
                switch (key.toString())
                {
                    case "WaitCommand":
                        inputCommandGroup.addCommands(new WaitCommand(Long.parseLong(parameter)));
                        break;
                    case "TrajFollowCommand":
                        inputCommandGroup.addCommands(new TrajectoryFollowerCommand(m_driveSubsystem, parameter));
                        break;
                    case "ClawCommand":
                        if(parameter.toLowerCase(Locale.ROOT) == "open")
                        {
                            inputCommandGroup.addCommands(new InstantCommand(() -> {m_clawSubsystem.open();}));
                        }
                        else if(parameter.toLowerCase(Locale.ROOT) == "close")
                        {
                            inputCommandGroup.addCommands(new InstantCommand(() -> {m_clawSubsystem.close();}));
                        }
                        break;
                    case "SlideTransitionCommand":
                        if(parameter.toLowerCase(Locale.ROOT) == "scoring")
                        {
                            inputCommandGroup.addCommands(new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.SCORING);}));
                        }
                        else if(parameter.toLowerCase(Locale.ROOT) == "junction")
                        {
                            inputCommandGroup.addCommands(new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.JUNCTIONLEVEL);}));
                        }
                        else if(parameter.toLowerCase(Locale.ROOT) == "ground")
                        {
                            inputCommandGroup.addCommands(new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.GROUNDLEVEL);}));
                        }
                        else if(parameter.toLowerCase(Locale.ROOT) == "acquired")
                        {
                            inputCommandGroup.addCommands(new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.ACQUIRED);}));
                        }
                        break;
                    case "SlideStateCommand":
                        params = parameter.split(" ");
                        if(params[0].toLowerCase(Locale.ROOT) == "junction")
                        {
                            inputCommandGroup.addCommands(new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.JUNCTIONLEVEL, Integer.getInteger(params[1]));}));
                        }
                        else if(params[0].toLowerCase(Locale.ROOT) == "stack")
                        {
                            inputCommandGroup.addCommands(new InstantCommand(() -> {m_linearSlideSubsystem.setState(Constants.LinearSlideState.STACKLEVEL, Integer.getInteger(params[1]));}));
                        }
                        break;
                    case "TurnCommand":
                        inputCommandGroup.addCommands(new TurnCommand(m_driveSubsystem, Math.toRadians(Double.parseDouble(parameter))));
                        break;
                    case "ParkingCommand":
                        params = parameter.split(" ");
                        inputCommandGroup.addCommands(new ParkingCommand(m_driveSubsystem, m_visionSubsystem,
                                params[0],
                                params[1],
                                params[2]));
                        break;
                    default:
                        break;
                }
            }
        }
        return inputCommandGroup;
    }

}
