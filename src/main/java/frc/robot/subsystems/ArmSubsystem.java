package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFX armMotorMaster = new TalonFX(Constants.ArmConstants.armMotorMaster);
    private final TalonFX armMotorSlave = new TalonFX(Constants.ArmConstants.armMotorSlave);
    private final Joystick joystick;

    private int shoulderLimit = 100000;

    public ArmSubsystem(Joystick joystick) {
        this.joystick = joystick;

        armMotorMaster.setSelectedSensorPosition(0);
        armMotorSlave.configFactoryDefault();

        armMotorMaster.configPeakOutputForward(0.2);
        armMotorMaster.configPeakOutputReverse(0.2);
        armMotorMaster.config_kP(0, 0.125); // kP .19 | kD .001 = 9829
        armMotorMaster.config_kD(0, 0.1);
        armMotorMaster.configAllowableClosedloopError(0, 100);
        armMotorMaster.setInverted(TalonFXInvertType.Clockwise);

        //armMotorSlave.configRemoteFeedbackFilter(9, RemoteSensorSource.TalonFX_SelectedSensor, 0);
        //armMotorSlave.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Joystick", -joystick.getY() * shoulderLimit);
        SmartDashboard.putNumber("Arm Master Falcon Position", armMotorMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Slave Falcon Position", armMotorSlave.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Slave Falcon Voltage", armMotorSlave.getMotorOutputVoltage());
        SmartDashboard.putNumber("Arm Master Falcon Voltage", armMotorMaster.getMotorOutputVoltage());
    }

    public void motorPositionControl(int position) {
        
    }

    public Command goToHome() {
        return runOnce(
            () -> {
                armMotorMaster.set(TalonFXControlMode.Position, 0);
                armMotorSlave.set(TalonFXControlMode.Position, 0);
            }
        );
    }

    public Command slowlyGoDown() {
        return runOnce(
            () -> {
                armMotorMaster.set(TalonFXControlMode.PercentOutput, -.1);
            }
        );
    }

    public Command slowyGoUp() {
        return runOnce(
            () -> {
                armMotorMaster.set(TalonFXControlMode.PercentOutput, .1);
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

    public Command setPosition() {
        return runOnce(
            () -> {
                double position = -joystick.getY() * shoulderLimit;
                if (position < 0) {
                    position = 0;
                }
                if (position < armMotorMaster.getSelectedSensorPosition()) {
                    armMotorMaster.setInverted(TalonFXInvertType.CounterClockwise);
                    armMotorSlave.setInverted(TalonFXInvertType.OpposeMaster);
                    armMotorMaster.set(TalonFXControlMode.Position, -position);
                    armMotorSlave.follow(armMotorMaster);
                  } else {
                    armMotorMaster.setInverted(TalonFXInvertType.Clockwise);
                    armMotorSlave.setInverted(TalonFXInvertType.OpposeMaster);
                    armMotorMaster.set(TalonFXControlMode.Position, position);
                    armMotorSlave.follow(armMotorMaster);
                  }
            }
        );
    }  
}
