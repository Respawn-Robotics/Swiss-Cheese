package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DefaultFalconConfigs;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    DefaultFalconConfigs config = new DefaultFalconConfigs();
    private final TalonFX armMotorMaster = new TalonFX(Constants.ArmConstants.ARM_MOTOR_MASTER);
    private final TalonFX armMotorSlave = new TalonFX(Constants.ArmConstants.ARM_MOTOR_SLAVE);

    public ArmSubsystem() {
        armMotorMaster.configFactoryDefault();
        config.SetDefaultStatusFrames(armMotorMaster);
        config.SetFollowerStatusFrames(armMotorSlave);
        
        armMotorMaster.setSelectedSensorPosition(0);
        armMotorSlave.setSelectedSensorPosition(0);

		armMotorMaster.config_kF(0, Constants.ArmConstants.UP_kF, 0);
		armMotorMaster.config_kP(0, Constants.ArmConstants.UP_kP, 0);
		armMotorMaster.config_kI(0, Constants.ArmConstants.UP_kI, 0);
		armMotorMaster.config_kD(0, Constants.ArmConstants.UP_kD, 0);

        armMotorMaster.config_kF(1, Constants.ArmConstants.DOWN_kF, 0);
		armMotorMaster.config_kP(1, Constants.ArmConstants.DOWN_kP, 0);
		armMotorMaster.config_kI(1, Constants.ArmConstants.DOWN_kI, 0);
		armMotorMaster.config_kD(1, Constants.ArmConstants.DOWN_kD, 0);

        armMotorMaster.configMotionSCurveStrength(8);

        armMotorMaster.setInverted(TalonFXInvertType.CounterClockwise);
        armMotorMaster.setNeutralMode(NeutralMode.Brake);
        armMotorSlave.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Master Falcon Position", armMotorMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Slave Falcon Position", armMotorSlave.getSelectedSensorPosition());

        // if(armMotorMaster.getBusVoltage() >= 10) {
        //     if(armMotorMaster.getStatorCurrent() > 25) {
        //         armMotorMaster.set(ControlMode.PercentOutput, 0);
        //     }
        // }

        // if(armMotorMaster.getBusVoltage() >= 8) {
        //     if(armMotorMaster.getStatorCurrent() > 22) {
        //         armMotorMaster.set(ControlMode.PercentOutput, 0);
        //     }
        // }

        // if(armMotorMaster.getBusVoltage() >= 6) {
        //     if(armMotorMaster.getStatorCurrent() > 17) {
        //         armMotorMaster.set(ControlMode.PercentOutput, 0);
        //     }
        // }
    }

    public TalonFX getMasterMotor() {
        return this.armMotorMaster;
    }

    public Command holdPosition() {
        return runOnce(
            () -> {
                armMotorMaster.set(TalonFXControlMode.MotionMagic, armMotorMaster.getSelectedSensorPosition());
                armMotorSlave.follow(armMotorMaster);
                armMotorSlave.setInverted(InvertType.FollowMaster);
            }
        );
    }

    public Command slowlyGoDown() {
        return runOnce(
            () -> {
                armMotorMaster.set(TalonFXControlMode.MotionMagic, armMotorMaster.getSelectedSensorPosition() - 500);
                armMotorSlave.follow(armMotorMaster);
                armMotorSlave.setInverted(InvertType.FollowMaster);
            }
        );
    }

    public Command slowyGoUp() {
        return runOnce(
            () -> {
                armMotorMaster.set(TalonFXControlMode.MotionMagic, armMotorMaster.getSelectedSensorPosition() + 500);
                armMotorSlave.follow(armMotorMaster);
                armMotorSlave.setInverted(InvertType.FollowMaster);
            }
        );
    }

    public Command stop() {
        return runOnce(
            () -> {
                armMotorMaster.set(TalonFXControlMode.PercentOutput, 0);
                armMotorSlave.set(TalonFXControlMode.PercentOutput, 0);
            }
        );
    }

    public Command resetSensor() {
        return runOnce(
            () -> {
                armMotorMaster.setSelectedSensorPosition(0);
                armMotorSlave.setSelectedSensorPosition(0);
            }
        );
    }

    public Command setPosition(double position) {
        return runOnce(
            () -> {
                    manageMotion(position);
                    armMotorMaster.set(ControlMode.MotionMagic, position);
                    armMotorSlave.follow(armMotorMaster);
                    armMotorSlave.setInverted(InvertType.FollowMaster);
            }
        );
    }

    public Command setVoltage(float voltage) {
        return runOnce(
            () -> {
                armMotorMaster.set(ControlMode.PercentOutput, voltage);
                armMotorSlave.follow(armMotorMaster);
                armMotorSlave.setInverted(InvertType.FollowMaster);
            }
        );
    }
    
    public void manageMotion(double targetPosition) {
        double currentPosition = armMotorMaster.getSelectedSensorPosition();
    
        // going up
        if(currentPosition < targetPosition) {
    
          // set accel and velocity for going up
          armMotorMaster.configMotionAcceleration(Constants.ArmConstants.CRUISE_VELOCITY_ACCEL_UP, 0);
          armMotorMaster.configMotionCruiseVelocity(Constants.ArmConstants.CRUISE_VELOCITY_ACCEL_UP, 0);
    
          // select the up gains
          armMotorMaster.selectProfileSlot(0, 0);
          SmartDashboard.putBoolean("Going Up or Down", true);
    
        } else {
          
          // set accel and velocity for going down
          armMotorMaster.configMotionAcceleration(Constants.ArmConstants.CRUISE_VELOCITY_ACCEL_DOWN, 0);
          armMotorMaster.configMotionCruiseVelocity(Constants.ArmConstants.CRUISE_VELOCITY_ACCEL_DOWN, 0);
    
          // select the down gains
          armMotorMaster.selectProfileSlot(1, 0);
          SmartDashboard.putBoolean("Going Up or Down", false);

        }
    
      }
}
