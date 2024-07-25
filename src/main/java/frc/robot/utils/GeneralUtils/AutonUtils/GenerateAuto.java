package frc.robot.utils.GeneralUtils.AutonUtils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.CommandUtils.SequentialGroupCommand;

public class GenerateAuto {
    /**
     * Generates a sequental command group for an auton or auton segment. 
     * @param autonName String: The name of the auton or auton segment.
     * @param autoCommands ArrayList<Command>: The arraylist of the commands to be placed in the sequental group command. 
     * @return SequentialGroupCommand: The command group that makes up the auton or auton segment. 
     */
    public static SequentialGroupCommand generateAuto(String autonName, ArrayList<Command> autoCommands) {
        Command[] commands = new Command[autoCommands.size()];
        for(int i = 0; i < autoCommands.size(); i++) {
            commands[i] = autoCommands.get(i);
        }
        
        return new SequentialGroupCommand(commands);
    }
}
