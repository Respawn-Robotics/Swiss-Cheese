package frc.robot.subsystems;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private double x,y,a,v;
  private final NetworkTable limelightTable;
  private ArrayList<Double> m_targetList;
  private final int MAX_ENTRIES = 50;
  private final NetworkTableEntry m_led_entry;
  private boolean pipeline = true;

  public LimelightSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_targetList = new ArrayList<Double>(MAX_ENTRIES);
    m_led_entry = limelightTable.getEntry("ledMode");
  }

  public double LimelightDistanceOffset(){
  NetworkTableEntry y = limelightTable.getEntry("ty");
  double targetOffsetAngle_Vertical = y.getDouble(0.0);
  
  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 0.0;
  
  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 8;
  
  // distance from the target to the floor
  double goalHeightInches = 23.6875;
  
  double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
  
  //calculate distance
  double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
  SmartDashboard.putNumber("DistanceOffset", distanceFromLimelightToGoalInches);
  SmartDashboard.putNumber("AngleOffset", angleToGoalDegrees);

  return distanceFromLimelightToGoalInches;
  }

  public CommandBase togglePipeline() {
    return runOnce(
        () -> {
          if(pipeline){
            System.out.println("Tape");
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
            pipeline = false;
          }else{
            System.out.println("Tag");
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
            pipeline = true;
          }
          NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
          NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        });
  }

  //pseudocode for team switch
  public Command TeamAprilTags() {
    return runOnce(
      () -> {
        
      }
    );
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

  @Override
  public void periodic() {
    x = limelightTable.getEntry("tx").getDouble(0);
    y = limelightTable.getEntry("ty").getDouble(0);
    a = limelightTable.getEntry("ta").getDouble(0);
    v = limelightTable.getEntry("tv").getDouble(0);

    LimelightDistanceOffset();
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", a);
    SmartDashboard.putNumber("LimeLightV", v);
  }
}