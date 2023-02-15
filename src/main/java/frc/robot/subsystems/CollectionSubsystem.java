package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class CollectionSubsystem extends SubsystemBase {
    private final DigitalInput collectionlimit = new DigitalInput(Constants.CollectionConstants.collectionBeamBreak);
    private final VictorSPX collectionMotor = new VictorSPX(Constants.CollectionConstants.collectionMotor);

    private boolean cubeLimitTouched;
    
    public CollectionSubsystem() {
      collectionMotor.setNeutralMode(NeutralMode.Brake);
    }
    @Override
    public void periodic() {
      SmartDashboard.putBoolean("CUBE BEAM BREAK", collectionlimit.get());
      cubeLimitTouched = !collectionlimit.get();

      
    }

    public Command collectCube() {
      return runOnce(
        () -> {
          if(!cubeLimitTouched) {
            collectionMotor.set(VictorSPXControlMode.PercentOutput, .5);
          } else {
            this.stopMotor();
          }
        }
      );
    }

    public Command ejectCube() {
      return runOnce(
        () -> {
          collectionMotor.set(VictorSPXControlMode.PercentOutput, -0.1);
        }
      );
    }
    
    public Command stopMotor() {
      return runOnce(
        () -> {
          collectionMotor.set(VictorSPXControlMode.PercentOutput, 0);
        }
      );
    }
    
    public Command collectCone() {
      return runOnce(
        () -> {
          collectionMotor.set(VictorSPXControlMode.PercentOutput, -.1);
        }
      );
    }
    
}
