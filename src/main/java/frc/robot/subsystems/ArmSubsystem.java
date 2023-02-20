package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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

    private int peakVelocityUp = 13360;
    private final double percentOfPeakUp = .65;
    private final double upkF = (percentOfPeakUp * 2048) / (peakVelocityUp * percentOfPeakUp);
    private final double cruiseVelocityAccelUp = peakVelocityUp * percentOfPeakUp;

    private int peakVelocityDown = 8090;
    private final double percentOfPeakDown = .65;
    private final double downkF = (percentOfPeakDown * 2048) / (peakVelocityDown * percentOfPeakDown);
    private final double cruiseVelocityAccelDown = peakVelocityDown * percentOfPeakDown;

    public ArmSubsystem(Joystick joystick) {
        this.joystick = joystick;
        armMotorMaster.configFactoryDefault();
        armMotorMaster.setSelectedSensorPosition(0);

		armMotorMaster.config_kF(0, 0.1, 0);
		armMotorMaster.config_kP(0, 0.06030624264, 0);
		armMotorMaster.config_kI(0, 0, 0);
		armMotorMaster.config_kD(0, 0, 0);

        armMotorMaster.config_kF(1, 0.1, 0);
		armMotorMaster.config_kP(1, 0.1265760198, 0);
		armMotorMaster.config_kI(1, 0, 0);
		armMotorMaster.config_kD(1, 0, 0);

        armMotorMaster.configMotionSCurveStrength(2);

        armMotorMaster.setInverted(TalonFXInvertType.Clockwise);
        armMotorMaster.setNeutralMode(NeutralMode.Brake);
        armMotorSlave.setNeutralMode(NeutralMode.Brake);

        //armMotorSlave.configRemoteFeedbackFilter(9, RemoteSensorSource.TalonFX_SelectedSensor, 0);
        //armMotorSlave.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Joystick", -joystick.getY() * shoulderLimit);
        SmartDashboard.putNumber("Arm Master Falcon Position", armMotorMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Slave Falcon Position", armMotorSlave.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Slave Falcon Voltage", armMotorSlave.getMotorOutputVoltage());
        SmartDashboard.putNumber("Arm Master Falcon Voltage", armMotorMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("Arm Up kF", upkF);
        SmartDashboard.putNumber("Arm up cruise velo + accel", cruiseVelocityAccelUp);
        SmartDashboard.putNumber("Arm Down kF", downkF);
        SmartDashboard.putNumber("Arm Down cruise velo + accel", cruiseVelocityAccelDown);
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
                armMotorSlave.follow(armMotorMaster);
                armMotorSlave.setInverted(InvertType.OpposeMaster);
            }
        );
    }

    public Command slowyGoUp() {
        return runOnce(
            () -> {
                armMotorMaster.set(TalonFXControlMode.PercentOutput, .1);
                armMotorSlave.follow(armMotorMaster);
                armMotorSlave.setInverted(InvertType.OpposeMaster);
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

    public Command pickUpOnGround() {
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
                    armMotorSlave.setInverted(InvertType.OpposeMaster);
            }
        );
    }

    public Command setVoltage(float voltage) {
        return runOnce(
            () -> {
                armMotorMaster.set(ControlMode.PercentOutput, voltage);
                armMotorSlave.follow(armMotorMaster);
                armMotorSlave.setInverted(InvertType.OpposeMaster);
            }
        );
    }
    
    public void manageMotion(double targetPosition) {
        double currentPosition = armMotorMaster.getSelectedSensorPosition();
    
        // going up
        if(currentPosition < targetPosition) {
    
          // set accel and velocity for going up
          armMotorMaster.configMotionAcceleration(cruiseVelocityAccelUp, 0);
          armMotorMaster.configMotionCruiseVelocity(cruiseVelocityAccelUp, 0);
    
          // select the up gains
          armMotorMaster.selectProfileSlot(0, 0);
          SmartDashboard.putBoolean("Going Up or Down", true);
    
        } else {
          
          // set accel and velocity for going down
          armMotorMaster.configMotionAcceleration(cruiseVelocityAccelDown, 0);
          armMotorMaster.configMotionCruiseVelocity(cruiseVelocityAccelDown, 0);
    
          // select the down gains
          armMotorMaster.selectProfileSlot(1, 0);
          SmartDashboard.putBoolean("Going Up or Down", false);

        }
    
      }
}
