// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.utils.GamepadUtils;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> _chooser = new SendableChooser<Command>();

  private double m_controllerDirection = -1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> {
            m_controllerDirection *= -1;
        }));

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    m_controllerDirection * GamepadUtils.squareInput(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                        -m_controllerDirection * GamepadUtils.squareInput(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                        m_controllerDirection * GamepadUtils.squareInput(
                        -m_driverController.getRightX(), OIConstants.kDriveDeadband)),
            m_robotDrive));

    // set the arm subsystem to run the "runAutomatic" function continuously when no other command
    // is running
    m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

    // set the intake to stop (0 power) when no other command is running
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

    // configure the launcher to stop when no other command is running
    m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));

    _chooser.setDefaultOption("shoot", this.getAutonomousShootCommand());
    _chooser.addOption("shoot and drive", this.getAutonomousShootAndDriveCommand());
    _chooser.addOption("2 note", this.getAutonomousTwoNote());
    SmartDashboard.putData(_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // set up arm preset positions
    new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));
    new Trigger(
            () ->
                m_operatorController.getLeftTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));

    //new JoystickButton(m_operatorController, XboxController.Button.kX.value)
    //    .onTrue(new InstantCommand(() -> m_arm.incrementEncoder(-1)));

    //new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
    //    .onTrue(new InstantCommand(() -> m_arm.incrementEncoder(1)));

    //intake controls (run while button is held down, run retract command once when the button is
    // relem 7]ased)
    new Trigger(
            () ->
                m_operatorController.getRightTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), m_intake))
        .onFalse(m_intake.retract());

    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(-1.0)));

    // launcher controls (button to pre-spin the launcher and button to launch)
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_launcher.runLauncher(), m_launcher));

    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .onTrue(m_intake.feedLauncher(m_launcher));

    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> m_launcher.launcherSpeed(-0.8), m_launcher));

        new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .onFalse(new RunCommand(() -> m_launcher.launcherSpeed(0), m_launcher));

    //new JoystickButton(m_operatorController, XboxController.Button.kX.value)
    //    .whileTrue(new RunCommand(() -> m_arm.runManual(.6), m_arm));
        
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return _chooser.getSelected();
  }

  private Command getAutonomousShootCommand() {
    var startLauncher = new StartEndCommand(() -> m_launcher.runLauncher(), () -> {}, m_launcher).withTimeout(1);
    var feedLauncher = m_intake.feedLauncher(m_launcher);

    return startLauncher.andThen(feedLauncher);
  }

  private Command getAutonomousShootAndDriveCommand() {
    return this.getAutonomousShootCommand().andThen(this.getAutonomousDriveForwardCommand());
  }

  private Command getAutonomousTwoNote() {
    var shoot = this.getAutonomousShootCommand();
    var driveToFirstNote = this.getAutonomousDriveForwardCommand();

    var driveBackToSpeaker = new StartEndCommand(
        () -> m_robotDrive.drive(.5,0,0),
        () -> m_robotDrive.drive(0, 0, 0),
        m_robotDrive).withTimeout(2); 

    var moveArmForIntake = new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition), m_arm);
    var intake = new StartEndCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), () -> m_intake.setPower(0), m_intake);
    var moveArmForScoring = new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition), m_arm);
    var shoot2 =  m_intake.feedLauncher(m_launcher);

    return new SequentialCommandGroup(
        shoot,
        moveArmForIntake,
        new ParallelCommandGroup(driveToFirstNote, intake),
        moveArmForScoring,
        driveBackToSpeaker,
        shoot2
    );
  }

  private Command getAutonomousDriveForwardCommand() {
    return new StartEndCommand(
        () -> m_robotDrive.drive(-.5,0,0),
        () -> m_robotDrive.drive(0, 0, 0),
        m_robotDrive).withTimeout(2);
  }
}
