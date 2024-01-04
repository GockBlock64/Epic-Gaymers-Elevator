package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

public class SetVelocityCommand extends CommandBase {
    Elevator elevator;

    public SetVelocityCommand(Elevator elevator, double velocity) {
        this.elevator = elevator;
        elevator.setVelocity(velocity);
    }

    
}
