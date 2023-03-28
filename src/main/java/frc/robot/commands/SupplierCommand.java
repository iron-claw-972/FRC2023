package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
/**
 * Runs the given command when this command is initialized, and ends when it ends. 
 * Useful for commands that are not created yet because the constructor parameters are not available until initialization.
 */
public class SupplierCommand extends CommandBase {
    
    Supplier<Command> m_commandSupplier;
    Command m_command;

    /**
     * Runs the given command when this command is initialized, and ends when it ends. 
     * Useful for commands that are not created yet because the constructor parameters are not available until initialization.
     * 
     * @param commandSupplier A Supplier to the command to run
     * @param Subsystem all subsystems that may be required to run the command supplied
     */
    public SupplierCommand(Supplier<Command> commandSupplier, Subsystem... Subsystem){
        addRequirements(Subsystem);
        m_commandSupplier = commandSupplier;
    } 

    @Override
    public void initialize() {
        m_command = m_commandSupplier.get();
        m_command.initialize();
    }

    @Override
    public void execute() {
        m_command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_command.isFinished();
    }

}
