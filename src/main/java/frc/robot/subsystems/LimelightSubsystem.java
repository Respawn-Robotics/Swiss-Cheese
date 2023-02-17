// package frc.robot.subsystems;

// import edu.wpi.first.math.filter.MedianFilter;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants.LimelightConstants;

// public class LimelightSubsystem extends SubsystemBase {
    
//     private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
//     private final NetworkTable table = ntInstance.getTable("/limelight");

//     private final NetworkTableEntry yaw = table.getEntry("tx");
//     private final NetworkTableEntry pitch = table.getEntry("ty");
//     private final NetworkTableEntry lightMode = table.getEntry("ledMode");
//     private final NetworkTableEntry validTarget = table.getEntry("tv");
//     private final NetworkTableEntry skew = table.getEntry("ts");
//     private final NetworkTableEntry cameraMode = table.getEntry("camMode");
//     private final NetworkTableEntry pipeline = table.getEntry("pipeline");
//     private final NetworkTableEntry poseData = table.getEntry("camtran");
//     private final NetworkTableEntry planeDistance = table.getEntry("planeDistance");
//     private Pose2d averagePose;
//     private MedianFilter averagePoseXFilter;
//     private MedianFilter averagePoseYFilter;
//     private MedianFilter averagePoseRotFilter;

//     public LimelightSubsystem() 
//     {
//         yaw.setDefaultDouble(0);
//         pitch.setDefaultDouble(0);
//         lightMode.setDefaultNumber(0);
//         validTarget.setDefaultNumber(0);
//         skew.setDefaultDouble(0);
//         cameraMode.setDefaultNumber(0);
//         pipeline.setDefaultNumber(0);
//         poseData.setDefaultDoubleArray(new double[6]);

//        averagePoseXFilter = new MedianFilter(10);
//        averagePoseYFilter = new MedianFilter(10);
//        averagePoseRotFilter = new MedianFilter(10);
//     }

//     @Override
//     public void periodic()
//     {
//         if (getPipeline() == LimelightConstants.PIPELINE_GET_POS) {
//             Pose2d currentPose = getPose();
//             if (currentPose != null) {
//                 averagePose = computeAveragePose(currentPose);
//             }
//         }
//         planeDistance.setDouble(getPlaneDistance());
//     }

//     public double getYaw() {
//         return this.yaw.getDouble(0);
//     }

//     public double getPitch() {
//         return pitch.getDouble(0);
//     }

//     public int getLightMode() {
//         return lightMode.getNumber(0).intValue();
//     }

//     public void setLEDMode(int ledMode) {
//         this.lightMode.setNumber(ledMode);
//     }

//     public boolean hasValidTarget() {
//         return validTarget.getNumber(0).intValue() == 1;
//     }

//     public double getSkew() {
//         return skew.getDouble(0);
//     }

//     public int getCameraMode() {
//         return cameraMode.getNumber(0).intValue();
//     }

//     public void setCameraMode(int cameraMode) {
//         this.cameraMode.setNumber(cameraMode);
//     }

//     public int getPipeline() {
//         return pipeline.getNumber(0).intValue();
//     }

//     public void setPipeline(int pipeline) {
//         this.pipeline.setNumber(pipeline);
//     }

//     /**
//      * @return The plane distance (along horizonntal plane) to the target or -1 if no target is found.
//      */
//     public double getPlaneDistance() {
//         if (!hasValidTarget()) {
//             return -1;
//         } 

//         return (LimelightConstants.TARGET_ELEVATION - LimelightConstants.CAMERA_ELEVATION)
//                 / Math.tan(Math.toRadians(LimelightConstants.CAMERA_ANGLE + getPitch()));
//     }

//     /** @return The distance to the target or -1 if no target is found. */
//     public double getDistance() {
//         if (!hasValidTarget()) {
//             return -1;
//         }

//         return (LimelightConstants.TARGET_ELEVATION - LimelightConstants.CAMERA_ELEVATION)
//                 / Math.sin(Math.toRadians(LimelightConstants.CAMERA_ANGLE + getPitch()));
//     }

//     private Pose2d getPose() 
//     {
//         if (getPipeline() != LimelightConstants.PIPELINE_GET_POS) {
//             throw new IllegalArgumentException(
//                     "The limelight must be set to the right pipeline to get run getPose()");
//         }

//         if (!hasValidTarget()) {
//             return null;
//         }

//         double[] data = poseData.getDoubleArray(new double[6]);
//         Rotation2d rot = Rotation2d.fromDegrees((data[4] + 180) % 360);
//         // TODO: figure out what "15" is. Maybe distance of center of robot to limelight?
//         double x = 15 * Math.cos(rot.getDegrees()) - data[2];
//         double y = 15 * Math.cos(90 - rot.getDegrees()) + data[0];
//         return new Pose2d(x, y, rot);
//     }

//     private Pose2d computeAveragePose(Pose2d newPose) {
        
//         double averageX = averagePoseXFilter.calculate(newPose.getTranslation().getX());
//         double averageY = averagePoseYFilter.calculate(newPose.getTranslation().getY());
//         double averageRot = averagePoseRotFilter.calculate(newPose.getRotation().getRadians());

//         return new Pose2d(averageX, averageY, new Rotation2d(averageRot));
//     }

//     public Pose2d getAveragePose() {
//         return averagePose;
//     }
// }