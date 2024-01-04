package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Elevator.*;

/**
 * An elevator subsystem that uses motors and encoders to move up and down
 */
public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged(); 

  private LoggedDashboardNumber LogP;
  private LoggedDashboardNumber LogI;
  private LoggedDashboardNumber LogD;
  private LoggedDashboardNumber LogFF;

  private MechanismLigament2d elevatorMechanism;

  private double setpoint = 0; // The target setpoint to move the elevator to

  public Elevator(ElevatorIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);

    LogP = new LoggedDashboardNumber("Elevator/P", io.getP());
    LogI = new LoggedDashboardNumber("Elevator/I", io.getI());
    LogD = new LoggedDashboardNumber("Elevator/D", io.getD());
    LogFF = new LoggedDashboardNumber("Elevator/FF", io.getFF());
  }

  /*
  public double highSetpoint() {
    return ELEVATOR_MAX_HEIGHT;
  }
  */
  
  public static enum States {
    Manual,
    PID
  }

  @Override
  public void periodic() { // constant loop
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

  /**
   * Moves the elevator at a given voltage
   * @param in the voltage to move the elevator at
   * @return whether or not the elevator is being pushed past its limit
   */
  public boolean move(double in) {
    boolean atLimit = false;
    if (io.getPositionMeters() > ELEVATOR_MAX_HEIGHT && in > 0) {
      in = 0;
      atLimit = true;
    } else if (io.getPositionMeters() < ELEVATOR_MIN_HEIGHT && in < 0) {
      in = 0;
      atLimit = true;
    }
    io.setVoltage(in * 12);
    return atLimit;
  }
  
  /**
   * Set a setpoint for PID. Does not start PID.
   * @param setpoint height to move to
   */
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }
  /**
   * Returns true if the elevator has reached the target setpoint
   */
  public boolean atSetpoint() {
    return Math.abs(io.getPositionMeters() - setpoint) < ELEVATOR_TOLERANCE;
  }
  /**
   * @return Simulation MechanismLigament2d representing the elevator arm
   */
  public MechanismLigament2d getElevatorMechanism() {
    return new MechanismLigament2d("Elevator", 5, 36, 5, new Color8Bit(Color.kOrange));
  }

  public void startPID() {
    io.setSetpoint(setpoint);
  }

  public void endPID() {
    io.setVelocity(0);
  }
  
  /**
   * @return A PIDCommand that moves the elevator to its max height using PID
   */
  public Command PIDCommandMax(){
    return PIDCommand(ELEVATOR_MAX_HEIGHT);
  }

  /**
   * @return A PIDCommand that moves the elevator to its min height using PID
   */
  public Command PIDCommandMin(){
    return PIDCommand(ELEVATOR_MIN_HEIGHT);
  }
  
  /**
   * Get a command to move the elevator to a setpoint using PID
   * @param setpoint the height to move to
   * @return returns FunctionalCommand that moves elevator to setpoint
   */
  public Command PIDCommand(double setpoint) {
    return new FunctionalCommand(
        () -> setSetpoint(setpoint),
        () -> startPID(),
        (stop) -> endPID(),
        this::atSetpoint,
        this);
  }
}