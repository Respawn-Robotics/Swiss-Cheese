package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    
    private final TalonFX wristMotor = new TalonFX(Constants.WristConstants.wristMotor);
    private final Joystick joystick;

    private int wristLimit = 80000;
    private int gravityFeedforward = 0;

    public WristSubsystem(Joystick joystick) {
        this.joystick = joystick;
        
        wristMotor.setSelectedSensorPosition(0);
        wristMotor.configPeakOutputForward(1);
        wristMotor.configPeakOutputReverse(1);
        wristMotor.configClosedLoopPeakOutput(0, 0.4);
        wristMotor.config_kP(0, 0.125); // kP .19 | kD .001 = 9829
        wristMotor.config_kD(0, 0.1);
        wristMotor.configAllowableClosedloopError(0, 0);
        wristMotor.setInverted(TalonFXInvertType.Clockwise);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Joystick", -joystick.getY() * wristLimit);
        SmartDashboard.putNumber("Wrist Falcon Position", wristMotor.getSelectedSensorPosition());

        int kMeasuredPosHorizontal = 0; 
        double kTicksPerDegree = Conversions.falconToDegrees(wristMotor.getSelectedSensorPosition(), 148);
        double currentPos = wristMotor.getSelectedSensorPosition();
        double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
        double radians = java.lang.Math.toRadians(degrees);
        double cosineScalar = java.lang.Math.cos(radians);

        // wristMotor.set(ControlMode.PercentOutput, gravityFeedforward * cosineScalar);
        wristMotor.setNeutralMode(NeutralMode.Brake);

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

    public Command stop() {
        return runOnce(
            () -> {
                wristMotor.set(ControlMode.PercentOutput, 0);
            }
        );
    }

    public Command setPosition() {
        return runOnce(
            () -> {
                double position = -joystick.getY() * wristLimit;
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
