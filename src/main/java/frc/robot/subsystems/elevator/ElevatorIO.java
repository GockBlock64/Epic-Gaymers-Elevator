package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;


public interface ElevatorIO {

    /**
     * A class that contains all the inputs for the elevator
     */
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    public default void updateInputs(ElevatorIOInputs inputs) {

    }

    public default double getPositionMeters() {
        return 0.0;
    } //Position of the elevator

    public default double getVelocityMetersPerSec() {
        return 0.0;
    } //Velocity of the elevator

    public default void zero() {

    } // Zeroes the motors (sets the current motor position to 0)

    public default void setSetpoint(double setpoint) {
        
    } //Target position the motor should travel to
    
    public default void setVoltage(double volts){

    }

    public default void setVelocity(double velocity) {}

    public default void setP(double kP){}
    public default void setI(double kI){}
    public default void setD(double kD){}

    public default double getP(){return 0.0;}
    public default double getI(){return 0.0;}
    public default double getD(){return 0.0;}  
}