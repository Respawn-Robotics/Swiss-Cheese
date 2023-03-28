package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DefaultFalconConfigs;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    DefaultFalconConfigs config = new DefaultFalconConfigs();

    private final TalonFX wristMotor = new TalonFX(Constants.WristConstants.WRIST_MOTOR);

    public WristSubsystem() {
        
        wristMotor.configFactoryDefault();

        config.SetDefaultStatusFrames(wristMotor);

        wristMotor.setSelectedSensorPosition(0);

		wristMotor.config_kF(0, Constants.WristConstants.UP_kF, 0);
		wristMotor.config_kP(0, Constants.WristConstants.UP_kP, 0);
		wristMotor.config_kI(0, Constants.WristConstants.UP_kI, 0);
		wristMotor.config_kD(0, Constants.WristConstants.UP_kD, 0);

        wristMotor.config_kF(1, Constants.WristConstants.DOWN_kF, 0);
		wristMotor.config_kP(1, Constants.WristConstants.DOWN_kP, 0);
		wristMotor.config_kI(1, Constants.WristConstants.DOWN_kI, 0);
		wristMotor.config_kD(1, Constants.WristConstants.DOWN_kD, 0);

        wristMotor.configMotionSCurveStrength(8);
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Falcon Position", wristMotor.getSelectedSensorPosition());

        // if(wristMotor.getBusVoltage() >= 12) {
        //     if(wristMotor.getStatorCurrent() > 36) {
        //         wristMotor.set(ControlMode.PercentOutput, 0);
        //     }
        // }

        // if(wristMotor.getBusVoltage() >= 10) {
        //     if(wristMotor.getStatorCurrent() > 31) {
        //         wristMotor.set(ControlMode.PercentOutput, 0);
        //     }
        // }

        // if(wristMotor.getBusVoltage() >= 8) {
        //     if(wristMotor.getStatorCurrent() > 23) {
        //         wristMotor.set(ControlMode.PercentOutput, 0);
        //     }
        // }
    }

    public TalonFX getMotor() {
        return wristMotor;
    }

    public Command holdPosition() {
        return runOnce(
            () -> {
                wristMotor.set(TalonFXControlMode.MotionMagic, wristMotor.getSelectedSensorPosition());
            }
        );
    }

    public Command slowlyGoDown() {
        return runOnce(
            () -> {
                wristMotor.set(TalonFXControlMode.MotionMagic, wristMotor.getSelectedSensorPosition() - 500);
            }
        );
    }

    public Command slowyGoUp() {
        return runOnce(
            () -> {
                wristMotor.set(TalonFXControlMode.MotionMagic, wristMotor.getSelectedSensorPosition() + 500);
            }
        );
    }

    public Command resetPos() {
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

    public Command setPosition(float position) {
        return runOnce(
            () -> {
                manageMotion(position);
                wristMotor.set(ControlMode.MotionMagic, position);
            }
        );
    }   

    public Command setVoltage(float voltage) {
        return runOnce(
            () -> {
                wristMotor.set(ControlMode.PercentOutput, voltage);
            }
        );
    }

    public void manageMotion(double targetPosition) {
        double currentPosition = wristMotor.getSelectedSensorPosition();
    
        // going up
        if(currentPosition < targetPosition) {
    
          // set accel and velocity for going up
          wristMotor.configMotionAcceleration(Constants.WristConstants.CRUISE_VELOCITY_ACCEL_UP, 0);
          wristMotor.configMotionCruiseVelocity(Constants.WristConstants.CRUISE_VELOCITY_ACCEL_UP, 0);
    
          // select the up gains
          wristMotor.selectProfileSlot(0, 0);
    
        } else {
          
          // set accel and velocity for going down
          wristMotor.configMotionAcceleration(Constants.WristConstants.CRUISE_VELOCITY_ACCEL_DOWN, 0);
          wristMotor.configMotionCruiseVelocity(Constants.WristConstants.CRUISE_VELOCITY_ACCEL_DOWN, 0);
    
          // select the down gains
          wristMotor.selectProfileSlot(1, 0);
        }
    
      }
}
