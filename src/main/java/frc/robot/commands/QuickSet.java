// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.CombinedControlConstants;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Elevator;

// public class QuickSet extends Command {
// private final Arm arm;
// private final Elevator elevator;
// private final PIDController leftController;
// private final PIDController rightController;
// private final PIDController AController;
// private final XboxController joystick;

// private Rotation2d targetAngle;
// private double targetPosition;
// private double lastSpeedL = 0;
// private double lastSpeedR = 0;

// public QuickSet(Elevator elevator, Arm arm, XboxController joystick) {
// this.elevator = elevator;
// this.arm = arm;
// this.joystick = joystick;

// this.AController = new PIDController(
// CombinedControlConstants.ARM_KP,
// CombinedControlConstants.ARM_KI,
// CombinedControlConstants.ARM_KD);
// this.leftController = new PIDController(
// ElevatorConstants.ELEVATOR_KP,
// ElevatorConstants.ELEVATOR_KI,
// ElevatorConstants.ELEVATOR_KD);
// this.rightController = new PIDController(
// ElevatorConstants.ELEVATOR_KP,
// ElevatorConstants.ELEVATOR_KI,
// ElevatorConstants.ELEVATOR_KD);

// leftController.setIntegratorRange(-0.5, 0.5);
// rightController.setIntegratorRange(-0.5, 0.5);

// addRequirements(elevator, arm);
// }

// @Override
// public void initialize() {
// targetPosition = elevator.getLeftPosition();
// targetAngle = arm.getAngle();

// elevator.setTargetPosition(targetPosition); // 確保 Elevator 內部的 PID 設置
// AController.setSetpoint(targetAngle.getRadians());

// }

// @Override
// public void execute() {

// if (joystick.getRawButtonPressed(7)) {
// elevator.setElevatorSpeed(0, 0);
// leftController.reset();
// rightController.reset();
// arm.stopArm();
// return; // 直接跳出，不執行後續 PID 控制
// }

// // **當其他按鈕被按下時，變更 targetPosition**
// if (joystick.getXButtonPressed()) {
// targetPosition = ElevatorConstants.ELEVATOR_DEFAULT_HEIGHT;
// } else if (joystick.getBButtonPressed()) {
// targetPosition = ElevatorConstants.ELEVATOR_CORAL_STATION_HEIGHT;
// }

// double leftOutput = MathUtil.clamp(
// leftController.calculate(elevator.getLeftPosition()),
// lastSpeedL - ElevatorConstants.ELEVATOR_MAX_ACCELERATION,
// lastSpeedL + ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
// double rightOutput = MathUtil.clamp(
// rightController.calculate(elevator.getRightPosition()),
// lastSpeedR - ElevatorConstants.ELEVATOR_MAX_ACCELERATION,
// lastSpeedR + ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
// double armOutput = MathUtil.clamp(
// AController.calculate(arm.getAngle().getRadians()),
// -CombinedControlConstants.ARM_SPEED,
// CombinedControlConstants.ARM_SPEED);

// lastSpeedL = leftOutput;
// lastSpeedR = rightOutput;

// elevator.setElevatorSpeed(-leftOutput, rightOutput);
// arm.setArmSpeed(armOutput);
// }

// @Override
// public boolean isFinished() {
// return Math.abs(targetPosition - elevator.getLeftPosition()) <
// ElevatorConstants.ELEVATOR_TOLERANCE &&
// Math.abs(targetPosition - elevator.getRightPosition()) <
// ElevatorConstants.ELEVATOR_TOLERANCE &&
// Math.abs(targetAngle.getRadians()
// - arm.getAngle().getRadians()) < CombinedControlConstants.ARM_TOLERANCE;
// }

// @Override
// public void end(boolean interrupted) {
// elevator.setElevatorSpeed(0, 0);
// arm.setArmSpeed(0);
// }
// }