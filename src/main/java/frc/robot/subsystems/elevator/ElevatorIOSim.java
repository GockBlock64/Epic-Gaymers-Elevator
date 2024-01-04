package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

import static frc.robot.Constants.Elevator.*;

public class ElevatorIOSim implements ElevatorIO {

  private DCMotor kGearbox = DCMotor.getVex775Pro(4);

  
  private ElevatorSim sim;
  private PIDController PIDController;
  private Encoder encoder;
  private EncoderSim encoderSim;

  public ElevatorIOSim() {
    // Magic constants lmao
    sim = new ElevatorSim(kGearbox, GEAR_RATIO, kCarriageMass, kDrumRadiusMeters, kMinHeightMeters, kMaxHeightMeters, false, VecBuilder.fill(0.01));
    PIDController = new PIDController(ELEVATOR_PID_SIM[0], ELEVATOR_PID_SIM[1], ELEVATOR_PID_SIM[2]);
    encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
    encoder.setDistancePerPulse(kEncoderDistPerPulse);
    encoderSim = new EncoderSim(encoder);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02);
    // Idk what the logic is here but i copied from DriveIOSim
    inputs.positionMeters = sim.getPositionMeters();
    encoderSim.setDistance(sim.getPositionMeters());
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public double getPositionMeters() {
    return sim.getPositionMeters();
  }

  @Override
  public double getVelocityMetersPerSec() {
    return sim.getVelocityMetersPerSecond();
  }

  @Override
  public void zero() {
  }
  
  @Override
  public void setSetpoint(double setpoint) {
    PIDController.setSetpoint(setpoint);
    setVoltage(PIDController.calculate(sim.getPositionMeters()));
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  @Override
  public void setVelocity(double velocity) {
    // More magic that I don't know how the frick it works
    PIDController.setSetpoint(velocity);
    sim.setInputVoltage(PIDController.calculate(sim.getPositionMeters()));
  }

  @Override
  public double getP() {
      return ELEVATOR_PID_SIM[0];
  }
  @Override
  public double getI() {
      return ELEVATOR_PID_SIM[1];
  }
  @Override
  public double getD() {
      return ELEVATOR_PID_SIM[2];
  }

  @Override
  public void setP(double kP) {
      ELEVATOR_PID_SIM[0] = kP;
  }
  @Override
  public void setI(double kI) {
      ELEVATOR_PID_SIM[1] = kI;
  }
  @Override
  public void setD(double kD) {
      ELEVATOR_PID_SIM[2] = kD;
  }
}
