package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    
    private final TalonFX wristMotor = new TalonFX(Constants.WristConstants.wristMotor);
    private final Joystick joystick;

    public WristSubsystem(Joystick joystick) {
        this.joystick = joystick;

        wristMotor.setSelectedSensorPosition(0);
        wristMotor.configPeakOutputForward(0.2);
        wristMotor.configPeakOutputReverse(0.2);
        wristMotor.config_kP(0, 0.125); // kP .19 | kD .001 = 9829
        wristMotor.config_kD(0, 0.1);
        wristMotor.configAllowableClosedloopError(0, 100);
        wristMotor.setInverted(TalonFXInvertType.Clockwise);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Joystick", -joystick.getY() * 10229);
        SmartDashboard.putNumber("Wrist Falcon Position", wristMotor.getSelectedSensorPosition());
    }

    public Command goToHome() {
        return runOnce(
            () -> {
                wristMotor.set(TalonFXControlMode.Position, 0);
            }
        );
    }

    public Command resetSensor() {
        return runOnce(
            () -> {
                wristMotor.setSelectedSensorPosition(0);
            }
        );
    }

    public Command setPosition() {
        return runOnce(
            () -> {
                double position = -joystick.getY() * 10229;
                if (position < 0) {
                    position = 0;
                }
                if (position < wristMotor.getSelectedSensorPosition()) {
                    wristMotor.setInverted(TalonFXInvertType.CounterClockwise);
                    wristMotor.set(TalonFXControlMode.Position, -position);
                  } else {
                    wristMotor.setInverted(TalonFXInvertType.Clockwise);
                    wristMotor.set(TalonFXControlMode.Position, position);
                  }
            }
        );
    }   
}
