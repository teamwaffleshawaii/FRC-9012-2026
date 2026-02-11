// package frc.robot.subsystems;


// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.IntakeConstants;
// import com.revrobotics.spark.*;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid;

// public class IntakeSubsystem extends SubsystemBase {

//     // --- Motors ---
//      public final SparkMax leftMotorIntake = new SparkMax(14, MotorType.kBrushless);

//      public final SparkMax rightMotorIntake = new SparkMax(13, MotorType.kBrushless);
    
//       public final SparkMax transferIntake = new SparkMax(17, MotorType.kBrushless);
//     // Pneumatics (REV Pneumatic Hub)
//     public  Compressor pCompressor = new Compressor(2,PneumaticsModuleType.REVPH);
//     public  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2,PneumaticsModuleType.REVPH, 0, 1);
    
//     public double speed = 0.1;

//     /** Set intake motor speed (-1 to 1). */
//     public void runIntake(double speed) {
//         leftMotorIntake.set(speed);
//         rightMotorIntake.set(-speed);
        
//     }

//     public void Transfer(double speed) {
//        transferIntake.set(speed);
//     }

//     /** Stop intake motors. */
//     public void stopIntake() {
//         leftMotorIntake.set(0);
//         rightMotorIntake.set(0);
//     }

//     /** Extend intake pivot. */
//     public void extendIntake() {
//        intakeSolenoid.set(Value.kForward);
//     }

//     /** Retract intake pivot. */
//     public void retractIntake() {
//         intakeSolenoid.set(Value.kReverse);
       
//     }

//     /** Toggle intake pivot state. */
//     public void toggleIntakePivot() {
//         if (intakeSolenoid.get() == Value.kForward) {
//             retractIntake();
//         } else {
//             extendIntake();
//         }
//     }
// }

    

