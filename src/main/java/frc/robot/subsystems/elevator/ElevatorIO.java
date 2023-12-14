package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public interface ElevatorIO {
    public enum State {
        MANUAL,
        PID
    }

    @AutoLog
    public static class ElevatorIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
    }

    public void updateInputs(ElevatorIOInputs inputs);

    public void setVoltage(double volts);

    public double getPositionMeters(); //Position of the elevator

    public double getVelocityMetersPerSec(); //Velocity of the elevator

    public void zero(); // Zeroes the motors (sets the current motor position to 0)

    public void setSetpoint(double setpoint); //Target position the motor should travel to
}