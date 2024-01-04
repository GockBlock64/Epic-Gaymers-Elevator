package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import static frc.robot.Constants.Elevator.TOLERANCE;
public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputs inputs = new ElevatorIOInputs();
    private double setpoint = 0;

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);

    }
    
    public void startPID() {
        io.startPID();
    }
    public void endPID() {
        io.endPID();
    }
    public void setSetpoint(double setpoint) {
        io.setSetpoint(setpoint);
    }
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public boolean atSetpoint() {
        return Math.abs(io.getPositionMeters() - setpoint) < TOLERANCE;
      }

      public Command PIDCommand(double setpoint) {
        return new FunctionalCommand(
            () -> setSetpoint(setpoint), 
            () -> startPID(), 
            (stop) -> endPID(), 
            this::atSetpoint, 
            this
        );
    }
}