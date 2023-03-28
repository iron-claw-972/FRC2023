package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SupplierCommand extends CommandBase {
    
    Supplier<Command> m_commandSupplier;
    Command m_command;

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
