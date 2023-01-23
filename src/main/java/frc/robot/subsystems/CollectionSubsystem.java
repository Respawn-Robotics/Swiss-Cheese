package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectionSubsystem extends SubsystemBase{
    private final DigitalInput limit = new DigitalInput(9);
    private final TalonFX falcon = new TalonFX(23);
  
    public Command limitMotor() {
        return runOnce(
          () -> {
            if(limit.get()) {
              System.out.println("LIMIT TOUCHES");
              falcon.set(TalonFXControlMode.PercentOutput, 0);
            } else {
              falcon.set(TalonFXControlMode.PercentOutput, 0.5);
            }
          }
        );
      }
    
      public Command runMotor() {
        return runOnce(
          () -> {
            
              falcon.set(TalonFXControlMode.PercentOutput, 0.5);
          }
        );
      }
    
      public Command disableMotor() {
        return runOnce(
          () -> {
              falcon.set(TalonFXControlMode.PercentOutput, 0);
          }
        );
      }
    
      public Command reject() {
        return runOnce(
          () -> {
              falcon.set(TalonFXControlMode.PercentOutput, -0.5);
          }
        );
      }
    
}
