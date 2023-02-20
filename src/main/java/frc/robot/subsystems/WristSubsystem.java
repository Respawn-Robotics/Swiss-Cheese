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

    private int peakVelocity = 14940;
    private final double percentOfPeak = .3;
    private final double kF = (percentOfPeak * 2048) / (peakVelocity * percentOfPeak);

    private final double cruiseVelocityAccel = peakVelocity * percentOfPeak;

    private final double gravityFeedforward = 0.05;

    private int measuredPosHorizontal = 50000; // Position measured shoulder arm is horizontal
    private int ticksPerDegree = (360 / (128 * 2048));

    public WristSubsystem(Joystick joystick) {
        this.joystick = joystick;
        
        wristMotor.configFactoryDefault();
        wristMotor.setSelectedSensorPosition(0);

        wristMotor.selectProfileSlot(0, 0);
		wristMotor.config_kF(0, .1, 0);
		wristMotor.config_kP(0, 0.0367156687, 0);
		wristMotor.config_kI(0, 0, 0);
		wristMotor.config_kD(0, 0, 0);

        wristMotor.configMotionAcceleration(cruiseVelocityAccel, 0); // 30% of max is 5515
        wristMotor.configMotionCruiseVelocity(cruiseVelocityAccel, 0);

        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Joystick", -joystick.getY() * wristLimit);
        SmartDashboard.putNumber("Wrist Falcon Position", wristMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Wrist kF", kF);
        SmartDashboard.putNumber("Wrist Vel + Accel", cruiseVelocityAccel);
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
                // double position = -joystick.getY() * wristLimit;
                // if (position < 0) {
                //     position = 0;
                // }

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
}
