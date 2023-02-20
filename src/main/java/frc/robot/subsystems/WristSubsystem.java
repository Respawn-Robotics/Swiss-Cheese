package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
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

    private int wristLimit = 184822;

    private int peakVelocityUp = 14940;
    private final double percentOfPeakUp = .95;
    private final double upkF = (percentOfPeakUp * 2048) / (peakVelocityUp * percentOfPeakUp);
    private final double cruiseVelocityAccelUp = peakVelocityUp * percentOfPeakUp;

    private int peakVelocityDown = 14940;
    private final double percentOfPeakDown = .65;
    private final double downkF = (percentOfPeakDown * 2048) / (peakVelocityDown * percentOfPeakDown);
    private final double cruiseVelocityAccelDown = peakVelocityDown * percentOfPeakDown;

    private final double gravityFeedforward = 0.05;

    private int measuredPosHorizontal = 50000; // Position measured shoulder arm is horizontal
    private int ticksPerDegree = (360 / (128 * 2048));

    public WristSubsystem(Joystick joystick) {
        this.joystick = joystick;
        
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
        SmartDashboard.putNumber("Joystick", -joystick.getY() * wristLimit);
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
