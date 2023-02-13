package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class CollectionSubsystem extends SubsystemBase {
    private final DigitalInput collectionlimit = new DigitalInput(Constants.CollectionConstants.collectionBeamBreak);
    private final VictorSPX collectionMotor = new VictorSPX(Constants.CollectionConstants.collectionMotor);
    
    public Command runMotor() {
      return runOnce(
        () -> {  
          collectionMotor.set(VictorSPXControlMode.PercentOutput, 0.5);
        }
      );
    }
    
    public Command disableMotor() {
      return runOnce(
        () -> {
          collectionMotor.set(VictorSPXControlMode.PercentOutput, 0);
        }
      );
    }
    
    public Command eject() {
      return runOnce(
        () -> {
          collectionMotor.set(VictorSPXControlMode.PercentOutput, -0.5);
        }
      );
    }
    
}
