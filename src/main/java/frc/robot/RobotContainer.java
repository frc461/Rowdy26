// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherCommand;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.FieldUtil;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.drivetrain.SwerveTelemetry;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.launcher.Launcher;

public class RobotContainer {

  private final AutoCommand autoCommand = new AutoCommand();

  public RobotContainer() {
    NamedCommands.registerCommand("Shoot Tower Preset", autoCommand.AutoTowerShoot());
    NamedCommands.registerCommand("Shoot Hub Preset", autoCommand.AutoHubShoot());
    NamedCommands.registerCommand("Stop All", autoCommand.StopAll());
    configureBindings();
  }

  private final Intake intake = new Intake();
  private final Launcher launcher = new Launcher();
  private final Spindexer spindexer = new Spindexer();

  private final TalonFX leadMotor = new TalonFX(50);//Spindexer
  private final TalonFX followMotor = new TalonFX(55);//Kicker

  private final CommandXboxController opjoystick = new CommandXboxController(1); // operator controller port

  private final CommandXboxController drjoystick = new CommandXboxController(0);

  

  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);


  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  public double rPM;


  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Left Trench shoot and Human Player");
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-drjoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-drjoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-drjoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    //AUTO TESTING BINDS - COMMENTED FOR SYSID TESTING
    // drjoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // drjoystick.b().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-drjoystick.getLeftY(), -drjoystick.getLeftX()))
    // ));
    // drjoystick.leftBumper().onTrue(new PathPlannerAuto("Spin"));
    // drjoystick.rightBumper().onTrue(new PathPlannerAuto("Spin"));

    // SignalLogger.setPath("/media/sda1/ctre-logs/");
    
    // drjoystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
  
    // drjoystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    


    /*
    * drjoystick Y = quasistatic forward
    * drjoystick A = quasistatic reverse
    * drjoystick B = dynamic forward
    * drjoystick X = dyanmic reverse
    */
    // drjoystick.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // drjoystick.a().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // drjoystick.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // drjoystick.x().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    //  Reset the field-centric heading on left bumper press.


    drjoystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    drjoystick.rightBumper().onTrue(
      Commands.runOnce(
        () -> launcher.setHoodPosition(0),
        launcher
      )
    );

    // LauncherCommand m_LauncherCommand = new LauncherCommand(launcher);
    // drjoystick.rightBumper().onTrue(m_LauncherCommand);


    // drivetrain.registerTelemetry(logger::telemeterize);

    opjoystick.rightBumper().whileTrue(
        Commands.startEnd(
          () -> intake.setIntakeVoltage(-16),
          () -> intake.setIntakeVoltage(0),
          intake
        )
    );

    // y sets presets for launcher and hood motor to shoot at hub
    
    opjoystick.y().onTrue(new InstantCommand(() -> {
          launcher.setFlywheelVelocity(-1950.0);
          launcher.setHoodPosition(0);
        }
      )
    );

    opjoystick.x().whileTrue(
      Commands.startEnd(
        () -> spindexer.setVoltage(-16),
        () -> spindexer.setVoltage(0),
        spindexer
      )
    );

    //  a sets presets for launcher and hood motor to shoot at tower
    
    opjoystick.a().onTrue(new InstantCommand(() -> {
          launcher.setFlywheelVelocity(-2250.0);
          launcher.setHoodPosition(1.25);
        }
      )
    );

    opjoystick.rightTrigger().whileTrue(
      Commands.startEnd(
        () -> {
          launcher.runFlyWheel();
          launcher.runHood();
        },
        () -> launcher.stopFlyWheels(),
        launcher
      )
    );

    opjoystick.b().whileTrue(
      Commands.sequence(
            Commands.run(
            () -> spindexer.setVoltage(-16),
            spindexer
        ).withTimeout(0.25),

                Commands.run(
            () -> spindexer.setVoltage(16),
            spindexer
        )
      )
    ).onFalse(
    Commands.runOnce(
        () -> spindexer.setVoltage(0),
        spindexer
        
      )
    );
    
    opjoystick.b().whileTrue(
      Commands.startEnd(
        () -> intake.setIntakeVoltage(16),
        () -> intake.setIntakeVoltage(0),
        intake
      )
    );
  }
  
  

}

