// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drive.SwerveJoystickCmd;
import frc.robot.Drive.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController addOnssController = new CommandXboxController(1);

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem
        , () -> driverController.getLeftX()
        , () -> driverController.getLeftY()
        , () -> driverController.getRightX()
        , null
        )
    );

    configureBindings();
  }

  private void configureBindings() {

    driverController.y().onTrue(new InstantCommand( ()-> swerveSubsystem.ZeroHeading() ));

  }

  public void mResetEncoders(){

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
