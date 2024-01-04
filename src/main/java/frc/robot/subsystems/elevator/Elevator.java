package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import static frc.robot.Constants.Elevator.*;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private LoggedDashboardNumber LogP;
  private LoggedDashboardNumber LogI;
  private LoggedDashboardNumber LogD;
  private LoggedDashboardNumber LogFF;

  private MechanismLigament2d elevatorMechanism;

  private double setpoint = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);

    LogP = new LoggedDashboardNumber("Elevator/P", io.getP());
    LogI = new LoggedDashboardNumber("Elevator/I", io.getI());
    LogD = new LoggedDashboardNumber("Elevator/D", io.getD());
    LogFF = new LoggedDashboardNumber("Elevator/FF", io.getFF());
  }

  public double highSetpoint() {
    return io.ELEVATOR_MAX_HEIGHT;
  }

  public static enum States {
    Manual,
    PID
  }

  @Override
  public void periodic() { //constant loop
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);

    elevatorMechanism.setLength(io.getPositionMeters());
    
    if (LogP.get() != io.getP()) 
      io.setP(LogP.get());
    
    if (LogI.get() != io.getI())
      io.setI(LogI.get());
    
    if (LogD.get() != io.getD())
      io.setD(LogD.get());
    
    if (LogFF.get() != io.getFF())
      io.setFF(LogFF.get());
    }

  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public boolean atSetpoint() {
        return Math.abs(io.getPositionMeters() - setpoint) < ELEVATOR_TOLERANCE;
    }

  public MechanismLigament2d getElevatorMechanism() {
        return new MechanismLigament2d("Elevator", 5, 36, 5, new Color8Bit(Color.kOrange));
    }
}