package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.Elevator;
public class ElevatorIOSim implements ElevatorIO {

    private PIDController controller = new PIDController(Elevator.kP, Elevator.kI, Elevator.kD, Elevator.kFF);
    //public ElevatorIOSim() {
    //   
    //}

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        
    }

    public void setSetpoint(double setpoint) {
        
    }
    
    public void setPIDFF(double kP, double kI, double kD, double kFF) {

    }

    @Override
    public void setVoltage(double volts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public double getPositionMeters() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPositionMeters'");
    }

    @Override
    public double getVelocityMetersPerSec() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocityMetersPerSec'");
    }

    @Override
    public void zero() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'zero'");
    }

    @Override
    public void startPID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'startPID'");
    }

    @Override
    public void endPID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'endPID'");
    }

}
