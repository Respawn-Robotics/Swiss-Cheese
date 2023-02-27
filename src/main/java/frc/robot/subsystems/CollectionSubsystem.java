package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectionSubsystem extends SubsystemBase {
    private final TalonSRX collectionMotor = new TalonSRX(Constants.CollectionConstants.collectionMotor);

    public CollectionSubsystem() {
      collectionMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Collection Motor Current", collectionMotor.getStatorCurrent());
      SmartDashboard.putNumber("Collection Motor Voltage", collectionMotor.getMotorOutputVoltage());
    }

    public Command collectCube() {
      return runOnce(
        () -> {
            collectionMotor.set(TalonSRXControlMode.PercentOutput, .50);
        }
      );
    }

    public Command puffCube() {
      return runOnce(
        () -> {
          collectionMotor.set(TalonSRXControlMode.PercentOutput, -1);
        }
      );
    }

    public Command shootCube() {
      return runOnce(
        () -> {
          collectionMotor.set(TalonSRXControlMode.PercentOutput, -0.5);
        }
      );
    }
    
    public Command stopMotor() {
      return runOnce(
        () -> {
          collectionMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
      );
    }
    
    public Command collectCone() {
      return runOnce(
        () -> {
          collectionMotor.set(TalonSRXControlMode.PercentOutput, -.9);     
        }
      );
    }

    public Command ejectCone() {
      return runOnce(
        () -> {
          collectionMotor.set(TalonSRXControlMode.PercentOutput, 1);
        }
      );
    }
}
