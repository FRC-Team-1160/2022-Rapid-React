package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
*/

// Subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

// Commands
import frc.robot.commands.climb.ClimbControl;
import frc.robot.commands.turret.TurretControl;
import frc.robot.commands.intake.MiddleIndexerControl;
import frc.robot.commands.intake.FinalIndexerControl;
import frc.robot.commands.intake.IntakeControl;
import frc.robot.commands.shoot.ShooterControl;
import frc.robot.commands.shoot.ShootDistance;
import frc.robot.commands.turret.TurnToAngle;
import frc.robot.commands.drive.DriveDistance;
import frc.robot.commands.drive.TurnAround;

/*
import frc.robot.commands.drive.Drive;
import frc.robot.commands.vision.LimelightCameraToggle;
import frc.robot.commands.vision.LimelightLightToggle;
import frc.robot.commands.vision.LimelightSnapshotToggle;
import frc.robot.commands.vision.LimelightStreamToggle;
*/

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    public final DriveTrain m_driveTrain = DriveTrain.getInstance(); 
    public final Shooter m_shooter = Shooter.getInstance();
    public final Intake m_intake = Intake.getInstance();
    public final Climber m_climber = Climber.getInstance();
    public final Turret m_turret = Turret.getInstance();
  
    // Controllers
    private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
    private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);
    private Joystick m_secondStick = new Joystick(OIConstants.secondStickPort);

    //auto routines
    private final Command m_shootDistance = 
      new ParallelCommandGroup(
        new ShootDistance(m_shooter).withTimeout(3.4),
        new TurnToAngle(m_turret).withTimeout(2.0),
        new SequentialCommandGroup(
          new IntakeControl(m_intake, -0.4 * 12).withTimeout(0.2),
          new MiddleIndexerControl(m_intake, -0.6 * 12).withTimeout(0.2),
          new MiddleIndexerControl(m_intake, 0 * 12).withTimeout(1.6),
          new FinalIndexerControl(m_intake, -0.65 * 12).withTimeout(0.7)
        )
      );

      private final Command m_shootIndoor = 
      new ParallelCommandGroup(
        new ShooterControl(m_shooter, -0.3 * 12).withTimeout(3.2),
        new SequentialCommandGroup(
          new IntakeControl(m_intake, -0.4 * 12).withTimeout(0.2),
          new MiddleIndexerControl(m_intake, -0.6 * 12).withTimeout(0.2),
          new MiddleIndexerControl(m_intake, 0 * 12).withTimeout(1.6),
          new FinalIndexerControl(m_intake, -0.65 * 12).withTimeout(0.7)
        )
      );

      private final Command m_initialIntake = 
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new IntakeControl(m_intake, -0.2 * 12).withTimeout(1.5),
          new IntakeControl(m_intake, -0.4 * 12).withTimeout(0.1)
        ),
        new MiddleIndexerControl(m_intake, -0.3 * 12).withTimeout(2.0)
      );

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands
      m_driveTrain.setDefaultCommand(new RunCommand(
        () -> m_driveTrain.tankDrive(0.6 * m_mainStick.getRawAxis(1), 0.6 * m_mainStick.getRawAxis(4), 0.6 * m_mainStick.getRawAxis(2), 0.6 * m_mainStick.getRawAxis(3)),
        m_driveTrain)
      );

      m_chooser.addOption("ShootDistance", m_shootDistance);

    }

    public Command getShootIndoor() {

      return new ParallelCommandGroup(
        new ShooterControl(m_shooter, -0.3 * 12).withTimeout(3.2),
        new SequentialCommandGroup(
          new IntakeControl(m_intake, -0.4 * 12).withTimeout(0.2),
          new MiddleIndexerControl(m_intake, -0.6 * 12).withTimeout(0.2),
          new MiddleIndexerControl(m_intake, 0 * 12).withTimeout(1.6),
          new FinalIndexerControl(m_intake, -0.65 * 12).withTimeout(0.7)
        )
      );
    }

    public Command getShootDistance() {

      return new ParallelCommandGroup(
        new TurnToAngle(m_turret).withTimeout(2),
        new ShootDistance(m_shooter).withTimeout(3.4),
        new SequentialCommandGroup(
          new IntakeControl(m_intake, -0.4 * 12).withTimeout(0.2),
          new MiddleIndexerControl(m_intake, -0.6 * 12).withTimeout(0.2),
          new MiddleIndexerControl(m_intake, 0 * 12).withTimeout(1.6),
          new FinalIndexerControl(m_intake, -0.65 * 12).withTimeout(0.7)
        )
      );
    }

    public Command getInitialIntake() {

      return new ParallelCommandGroup(
        new SequentialCommandGroup(
          new IntakeControl(m_intake, -0.2 * 12).withTimeout(1.5),
          new IntakeControl(m_intake, -0.4 * 12).withTimeout(0.1)
        ),
        new MiddleIndexerControl(m_intake, -0.3 * 12).withTimeout(2.0)
      );
    }

      
  
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
      
      // Config for main stick

      // Initial Intake
      new JoystickButton(m_mainStick, Button.kRightBumper.value)
        .whenPressed(
          new ParallelCommandGroup(
            new SequentialCommandGroup(
              new IntakeControl(m_intake, -0.2 * 12).withTimeout(1.5),
              new IntakeControl(m_intake, -0.4 * 12).withTimeout(0.1)
            ),
            new MiddleIndexerControl(m_intake, -0.3 * 12).withTimeout(2.0)
          )
          
        );
      //

      new JoystickButton(m_mainStick, Button.kLeftBumper.value)
        .whileHeld(
          new MiddleIndexerControl(m_intake, -0.3 * 12)
        );

      /*new JoystickButton(m_mainStick, Button.kY.value)
      .whileHeld(
        new IntakeControl(m_intake, -0.5 * 12)
      );*/

      // Reverse Intake
      new JoystickButton(m_mainStick, Button.kB.value)
      .whenPressed(
        new SequentialCommandGroup(
          new FinalIndexerControl(m_intake, 0.4 * 12).withTimeout(0.2),
          new MiddleIndexerControl(m_intake, 0.6 * 12).withTimeout(0.5),
          new IntakeControl(m_intake, 0.5 * 12).withTimeout(0.5)
        )
      );
      
      // Auto aim
      new JoystickButton(m_mainStick, Button.kA.value)
        .whenPressed(
          new TurnToAngle(m_turret).withTimeout(3.0)
        );

      // Drive an arbitrary distance (drives 125% inches over actual parameter)
      new JoystickButton(m_mainStick, Button.kX.value)
          .whenPressed(
            new DriveDistance(48, 0.04 * 12, m_driveTrain)
          );

      // Turn around
      new JoystickButton(m_mainStick, Button.kY.value)
        .whenPressed(
          new SequentialCommandGroup(
            this.getShootIndoor(), //replace with getShootDistance to shoot at hub instead of indoor (requires limelight)
            new TurnAround(180, 0.09 * 12, m_driveTrain),
            new DriveDistance(25, 0.04 * 12, m_driveTrain),
            new ParallelCommandGroup(
              new DriveDistance(10, 0.04 * 12, m_driveTrain),
              this.getInitialIntake()
            ),
            this.getInitialIntake(),
            new TurnAround(180, 0.09 * 12, m_driveTrain),
            this.getShootIndoor() //replace with getShootDistance to shoot at hub instead of indoor (requires limelight)
          )
        );

      //manual fire (no limelight)
      new JoystickButton(m_firstStick, 8)
        .whenPressed(
          new ParallelCommandGroup(
            new ShooterControl(m_shooter, -0.3 * 12).withTimeout(3.2),
            new SequentialCommandGroup(
              new IntakeControl(m_intake, -0.4 * 12).withTimeout(0.2),
              new MiddleIndexerControl(m_intake, -0.6 * 12).withTimeout(0.2),
              new MiddleIndexerControl(m_intake, 0 * 12).withTimeout(1.6),
              new FinalIndexerControl(m_intake, -0.65 * 12).withTimeout(0.7)
            )
          )
        );
      //

      //auto fire
      new JoystickButton(m_firstStick, 1)
        .whenPressed(
          new ParallelCommandGroup(
            new TurnToAngle(m_turret).withTimeout(2),
            new ShootDistance(m_shooter).withTimeout(3.4),
            new SequentialCommandGroup(
              new IntakeControl(m_intake, -0.4 * 12).withTimeout(0.2),
              new MiddleIndexerControl(m_intake, -0.6 * 12).withTimeout(0.2),
              new MiddleIndexerControl(m_intake, 0 * 12).withTimeout(1.6),
              new FinalIndexerControl(m_intake, -0.65 * 12).withTimeout(0.7)
            )
          )
        );

      // Reverse shooter
      new JoystickButton(m_firstStick, 9)
        .whileHeld(
          new ShooterControl(m_shooter, 0.5 * 12)
        );

      // Config for first stick

      // Turn turret complex to the RIGHT
      new JoystickButton(m_firstStick, 5)
        .whenHeld(
          new TurretControl(m_turret, 0.3 * 12)
        );

      // Turn turret complex to the LEFT
      new JoystickButton(m_firstStick, 4)
        .whenHeld(
          new TurretControl(m_turret, -0.3 * 12)
        );
      
      // Climber FIRST DIRECTION
      new JoystickButton(m_firstStick, 11)
        .whenHeld(
          new ClimbControl(m_climber, -0.4 * 12)
        );
      
      // Climber SECOND DIRECTION
      new JoystickButton(m_firstStick, 10)
        .whenHeld(
          new ClimbControl(m_climber, 0.4 * 12)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    
    public Command getAutonomousCommand() {
      return m_shootDistance;
    }
    
}