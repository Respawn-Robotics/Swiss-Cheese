// package frc.robot.autos;

// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.autos.autoCommands.exampleAuto;
// import frc.robot.subsystems.*;


// public class AutonChooser {
//     private static  Swerve s_Swerve;
//     private static  ArmSubsystem armSubsystem;
//     private static  WristSubsystem wristSubsystem;
//     private static  CollectionSubsystem collectionSubsystem;
//     private static  Vision limelightSubsystem;
    
//     //Positions
//     public static final SendableChooser<Double> posChooser = new SendableChooser<>();
    
//     public static final SendableChooser<Integer> colorChooser = new SendableChooser<>();

//     //AutoRoutines
//     private static final Command basicAuton = new exampleAuto(s_Swerve,  armSubsystem,  wristSubsystem,  collectionSubsystem,  limelightSubsystem);

//     private static final Command doNothing = new WaitCommand(10);


//     // A chooser for autonomous commands
//     public final static SendableChooser<Command> chooser = new SendableChooser<>();

//     public static void setupSelectors() {
//         colorChooser.setDefaultOption("Red", 0);
//         colorChooser.addOption("Blue", 1);

//         chooser.setDefaultOption("doNothing", doNothing);
//         chooser.addOption("Taxi Simple", basicAuton);
//     }

// }
