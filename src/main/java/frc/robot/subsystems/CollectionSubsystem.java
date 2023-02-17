package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class CollectionSubsystem extends SubsystemBase {
    private final DigitalInput cubeLimit = new DigitalInput(1);
    private final DigitalInput coneLimit = new DigitalInput(2);

    private final Joystick operator = new Joystick(1);

    private final TalonSRX collectionMotor = new TalonSRX(Constants.CollectionConstants.collectionMotor);

    private boolean cubeLimitTouched;
    private boolean coneLimitTouched;

    public CollectionSubsystem() {
      collectionMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
      SmartDashboard.putBoolean("CONE BEAM BREAK", coneLimit.get());
      SmartDashboard.putBoolean("CUBE BEAM BREAK", cubeLimit.get());

      SmartDashboard.putNumber("Collection Motor Current", collectionMotor.getStatorCurrent());
      cubeLimitTouched = !cubeLimit.get();
      coneLimitTouched = !coneLimit.get();

      if(collectionMotor.getStatorCurrent() < -40) {
        collectionMotor.set(ControlMode.PercentOutput, 0);
      }

      if((coneLimitTouched || cubeLimitTouched) && !(operator.getRawButtonPressed(10))) {
        collectionMotor.set(ControlMode.PercentOutput, 0);
      }
    }

    public Command collectCube() {
      return runOnce(
        () -> {
          if(!cubeLimitTouched) {
            collectionMotor.set(TalonSRXControlMode.PercentOutput, .50);
          } else {
            this.stopMotor();
          }
        }
      );
    }

    public Command ejectCube() {
      return runOnce(
        () -> {
          collectionMotor.set(TalonSRXControlMode.PercentOutput, -0.1);
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
          collectionMotor.set(TalonSRXControlMode.PercentOutput, -.65);     
        }
      );
    }

    public Command ejectCone() {
      return runOnce(
        () -> {
          collectionMotor.set(TalonSRXControlMode.PercentOutput, .1);
        }
      );
    }
    
}
