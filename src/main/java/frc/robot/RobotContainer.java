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
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherCommand;
import frc.robot.subsystems.localizer.Localizer;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.FieldUtil;
import frc.robot.util.vision.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drivetrain.AimAtHubCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.drivetrain.SwerveCommand;
import frc.robot.subsystems.drivetrain.SwerveTelemetry;
import frc.robot.subsystems.hubState.HubState;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.launcher.Launcher;

public class RobotContainer {

  private SendableChooser<Command> autoChooser;
  private final Intake intake = new Intake();
  private final Launcher launcher = new Launcher();
  private final Spindexer spindexer = new Spindexer();

  private final AutoCommand autoCommand = new AutoCommand(launcher, spindexer, intake);
  private static HubState hubState = new HubState();

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

  // Create Vision, passing it the CTRE drivetrain's vision method
  public final Vision m_vision = new Vision((pose, timestamp, stdDevs) -> {
      drivetrain.addVisionMeasurement(pose, timestamp, stdDevs);
  });

  // Pass both to the Localizer
  public final Localizer m_localizer = new Localizer(drivetrain, m_vision);  
    
    public RobotContainer() {
  
      autoChooser = new SendableChooser<>();

      NamedCommands.registerCommand("Shoot Trench Preset", autoCommand.AutoTrenchShoot());
      NamedCommands.registerCommand("Shoot Human Player Preset", autoCommand.AutoHumanPlayerShoot());
      NamedCommands.registerCommand("Shoot Tower Preset", autoCommand.AutoTowerShoot());
      NamedCommands.registerCommand("Shoot Hub Preset", autoCommand.AutoHubShoot());
      NamedCommands.registerCommand("Stop Launcher", autoCommand.StopLauncher());
      NamedCommands.registerCommand("Stop All", autoCommand.StopAll());
      NamedCommands.registerCommand("Extend Intake", autoCommand.ExtendIntake());
      NamedCommands.registerCommand("Retract Intake", autoCommand.RetractIntake());
  
      configureBindings();
  
      boolean isCompetition = true;
  
      // Build an auto chooser. This will use Commands.none() as the default option.
      // As an example, this will only show autos that start with "comp" while at
      // competition as defined by the programmer
      autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
      );
  
      SmartDashboard.putData("Auto Chooser", autoChooser);
      SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
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
    * drjoystick B = dynamic forward\
    * drjoystick X = dyanmic reverse
    */
    // drjoystick.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // drjoystick.a().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // drjoystick.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));   
    // drjoystick.x().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  
    //Driver Controller
  
      drjoystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

      drjoystick.rightTrigger().whileTrue(
          Commands.startEnd(
            () -> intake.setIntakeVoltage(16),
            () -> intake.setIntakeVoltage(0),
            intake
          )
      );
  
    // drivetrain.registerTelemetry(logger::telemeterize);
  
    drjoystick.rightBumper().onTrue(Commands.run(
    ()-> {
      launcher.setHoodPosition(0.0);
      launcher.runHood();
    },
    launcher));
  
    // drjoystick.leftTrigger().whileTrue(
    //     Commands.startEnd(
    //       () -> intake.setIntakeVoltage(16), 
    //       () -> intake.setIntakeVoltage(0),
    //       intake)
    // );
    
    drjoystick.leftTrigger().whileTrue(
      new AimAtHubCommand(
          drivetrain, 
          launcher, 
          m_localizer, 
          () -> -drjoystick.getLeftY() * MaxSpeed,   // Forward input
          () -> -drjoystick.getLeftX() * MaxSpeed    // Strafe input
        )
    );
  

    // Operator COntroller

    opjoystick.leftTrigger().whileTrue(
      Commands.startEnd(
        () ->launcher.shuttle(),
        ()->launcher.stopFlyWheels(),
        launcher
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

    opjoystick.leftBumper().onTrue(Commands.run(
      ()-> {
        launcher.setHoodPosition(0.0);
        launcher.runHood();
      },
      launcher));

    // opjoystick.leftBumper().onTrue(new InstantCommand(() -> {
    //         launcher.setFlywheelVelocity(Constants.LauncherConstants.TRENCH_AUTO_RPM);
    //         launcher.setHoodPosition(Constants.LauncherConstants.TRENCH_AUTO_START_HOOD_ANGLE);
    //       }
    //     )
    //     );

    opjoystick.rightBumper().whileTrue( 
      Commands.startEnd(
        () -> intake.setIntakeVoltage(-16),
        () -> intake.setIntakeVoltage(0),
        intake)
    );

    opjoystick.b().whileTrue(
      Commands.startEnd(
        () -> intake.setIntakeVoltage(-16),
        () -> intake.setIntakeVoltage(0),
        intake
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
    
    opjoystick.y().onTrue(new InstantCommand(() -> {
          launcher.setFlywheelVelocity(Constants.LauncherConstants.HUB_RPM);
          launcher.setHoodPosition(Constants.LauncherConstants.HUB_HOOD_ANGLE);
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
    
    opjoystick.a().onTrue(new InstantCommand(() -> {
          launcher.setFlywheelVelocity(Constants.LauncherConstants.TOWER_RPM);
          launcher.setHoodPosition(Constants.LauncherConstants.TOWER_HOOD_ANGLE);
        }
      )
    );  

    opjoystick.povUp().whileTrue(
      Commands.startEnd(
        () ->intake.setDeployVoltage(-6),
        () ->intake.setDeployVoltage(0),
        intake
      )
    );

    opjoystick.povDown().whileTrue(
      Commands.startEnd(
        () ->intake.setDeployVoltage(6),
        () ->intake.setDeployVoltage(0),
        intake
      )
    );
    
    opjoystick.povRight().onTrue(
      Commands.runOnce(
        () ->intake.ExtendIntake(),
        intake
      )
    );

    opjoystick.povLeft().onTrue(
      Commands.runOnce(
        () ->intake.RetractIntake(),
        intake
      )
    );
  }

  public Command getAutonomousCommand() {
    String selected = autoChooser.getSelected().getName();

    return autoChooser.getSelected();

    // if(selected.startsWith("comp LT")) {
    //   return new PathPlannerAuto(selected, true);
    // }
    // else {
    //   return autoChooser.getSelected();
    // }
  }  
}
