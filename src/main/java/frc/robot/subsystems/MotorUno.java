package frc.robot.subsystems;

/**------------------------------------------------------------------- */

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import java.util.function.DoubleSupplier;

public class MotorUno extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  final Talon arm_motor = new Talon(0);
  
  public CommandBase MoveMotorCommand(DoubleSupplier fractionalOutput) {
    return run(() -> arm_motor.set(fractionalOutput.getAsDouble()));
  }
  
  public CommandBase StopMotorCommand() {
    return run(() -> arm_motor.disable());
 
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation  Eric, "I don't think we even need this"
  }
}
