package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    encoder.setPositionConversionFactor(Math.PI * ELEVATOR_WHEEL_DIAM_M / ELEVATOR_GEARBOX_REDUCTION);
    encoder.setVelocityConversionFactor(Math.PI * ELEVATOR_WHEEL_DIAM_M / ELEVATOR_GEARBOX_REDUCTION / 60.0);

    encoder.setPosition(0);
  }
  
  
  private void configurePID() {
    PIDController.setP(kP);
    PIDController.setI(kI);
    PIDController.setD(kD);
    PIDController.setFF(kFF);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        encoder.getVelocity() / GEAR_RATIO);
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
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
}
