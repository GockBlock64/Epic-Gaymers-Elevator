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

}
