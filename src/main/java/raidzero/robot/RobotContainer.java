// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import raidzero.robot.commands.TeleopSwerve;
import raidzero.robot.subsystems.Swerve;

public class RobotContainer {
	// Controllers
	private final XboxController master = new XboxController(0);
	
	// Subsystems
	private final Swerve swerve = Swerve.getSwerve();

	public RobotContainer() {
		swerve.setDefaultCommand(new TeleopSwerve(
			() -> master.getLeftY(),
			() -> master.getLeftX(),
			() -> master.getRightX(),
			() -> false)
		);

		configureBindings();
	}

	private void configureBindings() {
		new JoystickButton(master, XboxController.Button.kY.value).onTrue(
			new InstantCommand(() -> swerve.zeroHeading())
		);
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
