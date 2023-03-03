package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectionSubsystem extends SubsystemBase {
    private final TalonFX collectionMotor = new TalonFX(Constants.CollectionConstants.collectionMotor);

    public CollectionSubsystem() {
      collectionMotor.configFactoryDefault();
      collectionMotor.setNeutralMode(NeutralMode.Brake);
      collectionMotor.setSelectedSensorPosition(0);

      collectionMotor.config_kF(0, 0, 0);
		  collectionMotor.config_kP(0, .2, 0);
		  collectionMotor.config_kI(0, 0, 0);
		  collectionMotor.config_kD(0, 0, 0);

      collectionMotor.config_kF(1, 0, 0);
		  collectionMotor.config_kP(1, .05, 0);
		  collectionMotor.config_kI(1, 0, 0);
		  collectionMotor.config_kD(1, 0, 0);
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Collection Motor Current", collectionMotor.getStatorCurrent());
      SmartDashboard.putNumber("Collection Motor Voltage", collectionMotor.getMotorOutputVoltage());
    }

    public TalonFX getMotor() {
      return collectionMotor;
    }

    public Command alterCube() {
      return runOnce(
        () -> {
            collectionMotor.set(ControlMode.PercentOutput, .1);
        }
      );
    }

    public Command collectCube() {
      return runOnce(
        () -> {
            collectionMotor.set(ControlMode.PercentOutput, .3);
        }
      );
    }

    public Command puffCube() {
      return runOnce(
        () -> {
          collectionMotor.set(ControlMode.PercentOutput, -0.1);
        }
      );
    }

    public Command shootCube() {
      return runOnce(
        () -> {
          collectionMotor.set(ControlMode.PercentOutput, -0.95);
        }
      );
    }
    
    public Command stopMotor() {
      return runOnce(
        () -> {
          collectionMotor.set(ControlMode.PercentOutput, 0);
        }
      );
    }

    public Command holdPosition() {
      return runOnce(
        () -> {
          collectionMotor.set(ControlMode.Position, collectionMotor.getSelectedSensorPosition());
          SmartDashboard.putNumber("HOLDING AT", collectionMotor.getSelectedSensorPosition());
        }
      );
    }
    
    public Command collectCone() {
      return runOnce(
        () -> {
          collectionMotor.set(ControlMode.PercentOutput, -.75);     
        }
      );
    }

    public Command ejectCone() {
      return runOnce(
        () -> {
          collectionMotor.set(ControlMode.PercentOutput, .70);
        }
      );
    }

    public Command setConeHoldingPressure() {
      return runOnce(
        () -> {
          collectionMotor.selectProfileSlot(0, 0);
        }
      );
    }

    public Command setCubeHoldingPressure() {
      return runOnce(
        () -> {
          collectionMotor.selectProfileSlot(1, 0);
        }
      );
    }
}
