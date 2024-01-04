package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.Elevator.*;

public class ElevatorIOSparkMax implements ElevatorIO {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private RelativeEncoder encoder;

  private SparkMaxPIDController PIDController;

  public ElevatorIOSparkMax() {
    leftMotor = new CANSparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

    PIDController = leftMotor.getPIDController();
    configureEncoders();
    configurePID();

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setInverted(false);
    rightMotor.follow(leftMotor, true);

    leftMotor.enableVoltageCompensation(12.0);
    leftMotor.setSmartCurrentLimit(30);
    leftMotor.burnFlash();
  }

  private void configureEncoders() {
    encoder = leftMotor.getEncoder();

    encoder.setPositionConversionFactor(Math.PI * ELEVATOR_ROTATION_DIAM_M / ELEVATOR_GEARBOX_REDUCTION);
    encoder.setVelocityConversionFactor(Math.PI * ELEVATOR_ROTATION_DIAM_M / ELEVATOR_GEARBOX_REDUCTION / 60.0);

    encoder.setPosition(0);
  }

  private void configurePID() {
    PIDController.setP(ELEVATOR_PID_REAL[0]);
    PIDController.setI(ELEVATOR_PID_REAL[1]);
    PIDController.setD(ELEVATOR_PID_REAL[2]);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionMeters = encoder.getPosition() * encoder.getPositionConversionFactor();
    inputs.velocityMetersPerSec = encoder.getVelocity() * encoder.getVelocityConversionFactor();
  }

  @Override
  public double getPositionMeters() {
    return encoder.getPosition() * encoder.getPositionConversionFactor();
  }

  @Override
  public double getVelocityMetersPerSec() {
    return encoder.getVelocity() * encoder.getVelocityConversionFactor();
  }

  @Override
  public void zero() {
    encoder.setPosition(0);
  }
  
  @Override
  public void setSetpoint(double setpoint) {
    PIDController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(MathUtil.clamp(volts,-12.0,12.0));
  }

  @Override
  public void setVelocity(double velocity) {
    leftMotor.set(velocity);
  }

  @Override
  public double getP() {
      return ELEVATOR_PID_REAL[0];
  }
  @Override
  public double getI() {
      return ELEVATOR_PID_REAL[1];
  }
  @Override
  public double getD() {
      return ELEVATOR_PID_REAL[2];
  }
  @Override
  public double getFF() {
      return ELEVATOR_PID_REAL[3];
  }

  @Override
  public void setP(double kP) {
      ELEVATOR_PID_REAL[0] = kP;
  }
  @Override
  public void setI(double kI) {
      ELEVATOR_PID_REAL[1] = kI;
  }
  @Override
  public void setD(double kD) {
      ELEVATOR_PID_REAL[2] = kD;
  }
  @Override
  public void setFF(double kFF) {
      ELEVATOR_PID_REAL[3] = kFF;
  }
}
