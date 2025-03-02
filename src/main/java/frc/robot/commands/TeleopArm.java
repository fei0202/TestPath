// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.CombinedControlConstants;
// import frc.robot.subsystems.Arm;

// public class TeleopArm extends Command {
//     private final Arm arm;
//     private final XboxController joystick1;
    
//     private double targetPosition; 
//     private final PIDController Acontroller = new PIDController(
//         CombinedControlConstants.ARM_KP, 
//         CombinedControlConstants.ARM_KI, 
//         CombinedControlConstants.ARM_KD
//     );

//     public TeleopArm(Arm arm, XboxController joystick1) {
//         this.arm = arm;
//         this.joystick1 = joystick1;
//         addRequirements(arm);
//     }

//     @Override
//     public void initialize() {
//         targetPosition = arm.getPosition();
//         Acontroller.setSetpoint(targetPosition);
//     }

//     @Override
//     public void execute() {
//         double manualSpeed = 0;

//         if (joystick1.getXButton()) {
//             manualSpeed = 0.2;
//             targetPosition = arm.getPosition();
//         } else if (joystick1.getBButton()) {
//             manualSpeed = -0.2;
//             targetPosition = arm.getPosition();
//         }

//         arm.setArmSpeed(manualSpeed);

//         SmartDashboard.putNumber("Arm Position", arm.getPosition());
//         SmartDashboard.putData("Arm Controller", Acontroller);
//     }

//     @Override
//     public boolean isFinished() {
//         return false; 
//     }

//     @Override
//     public void end(boolean interrupted) {
//         arm.setArmSpeed(0);
//     }
// }
