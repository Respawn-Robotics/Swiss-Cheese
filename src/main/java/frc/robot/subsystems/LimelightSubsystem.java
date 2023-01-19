package frc.robot.subsystems;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private double x,y,a,v;
  private final NetworkTable m_limelightTable;
  private ArrayList<Double> m_targetList;
  private final int MAX_ENTRIES = 50;
  private final NetworkTableEntry m_led_entry;

  private final DigitalInput limit = new DigitalInput(9);
  private final TalonFX falcon = new TalonFX(23);


  public LimelightSubsystem() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_targetList = new ArrayList<Double>(MAX_ENTRIES);
    m_led_entry = m_limelightTable.getEntry("ledMode");
  }

  public CommandBase RetroReflectiveTape() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public double getV() {
    return v;
  }

  public double getTX() {
    return x;
  }

  public double getTY() {
    return y;
  }

  public double getTA() {
    double sum = 0;

    for (Double num : m_targetList) { 		      
      sum += num.doubleValue();
    }
    return sum/m_targetList.size();
  }

  public boolean isTargetValid() {
    if(v == 1){
      return true;
    } else{
      return false;
    }
  }

  public Command setLedMode(int mode){
    return runOnce(
      () -> {
        m_led_entry.setDouble((mode));
      }
    );
  }

  public Command limitMotor() {
    return runOnce(
      () -> {
        if(limit.get()) {
          System.out.println("LIMIT TOUCHES");
          falcon.set(TalonFXControlMode.PercentOutput, 0);
        } else {
          falcon.set(TalonFXControlMode.PercentOutput, 0.5);
        }
      }
    );
  }

  public Command runMotor() {
    return runOnce(
      () -> {
        
          falcon.set(TalonFXControlMode.PercentOutput, 0.5);
      }
    );
  }

  public Command disableMotor() {
    return runOnce(
      () -> {
          falcon.set(TalonFXControlMode.PercentOutput, 0);
      }
    );
  }

  public Command reject() {
    return runOnce(
      () -> {
          falcon.set(TalonFXControlMode.PercentOutput, -0.5);
      }
    );
  }

  @Override
  public void periodic() {
    x = m_limelightTable.getEntry("tx").getDouble(0);
    y = m_limelightTable.getEntry("ty").getDouble(0);
    a = m_limelightTable.getEntry("ta").getDouble(0);
    v = m_limelightTable.getEntry("tv").getDouble(0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", a);
    SmartDashboard.putNumber("LimeLightV", v);
  }
}