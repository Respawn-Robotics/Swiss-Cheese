package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    
    private final TalonFX wristMotor = new TalonFX(Constants.WristConstants.wristMotor);

    private int peakVelocityUp = 14940;
    private final double percentOfPeakUp = .95;
    private final double cruiseVelocityAccelUp = peakVelocityUp * percentOfPeakUp;

    private int peakVelocityDown = 14940;
    private final double percentOfPeakDown = .95;
    private final double cruiseVelocityAccelDown = peakVelocityDown * percentOfPeakDown;

    public WristSubsystem() {
        
        wristMotor.configFactoryDefault();
        wristMotor.setSelectedSensorPosition(0);

		wristMotor.config_kF(0, .1, 0);
		wristMotor.config_kP(0, 0.0367156687, 0);
		wristMotor.config_kI(0, 0, 0);
		wristMotor.config_kD(0, 0, 0);

        wristMotor.config_kF(1, .1, 0);
		wristMotor.config_kP(1, 0.0367156687, 0);
		wristMotor.config_kI(1, 0, 0);
		wristMotor.config_kD(1, 0, 0);

        wristMotor.configMotionSCurveStrength(2);
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Falcon Position", wristMotor.getSelectedSensorPosition());
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
          wristMotor.configMotionAcceleration(cruiseVelocityAccelUp, 0);
          wristMotor.configMotionCruiseVelocity(cruiseVelocityAccelUp, 0);
    
          // select the up gains
          wristMotor.selectProfileSlot(0, 0);
    
        } else {
          
          // set accel and velocity for going down
          wristMotor.configMotionAcceleration(cruiseVelocityAccelDown, 0);
          wristMotor.configMotionCruiseVelocity(cruiseVelocityAccelDown, 0);
    
          // select the down gains
          wristMotor.selectProfileSlot(1, 0);
        }
    
      }
}
