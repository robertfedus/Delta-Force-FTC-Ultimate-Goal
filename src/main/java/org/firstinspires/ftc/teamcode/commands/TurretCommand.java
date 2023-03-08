package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RampSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {

    private final TurretSubsystem turretSubsystem;
    private final LocalizationSubsystem localizationSubsystem;
    private Telemetry tele;

    public TurretCommand(TurretSubsystem subsystem, LocalizationSubsystem lSubsystem) {
        turretSubsystem = subsystem;
        localizationSubsystem = lSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        turretSubsystem.setTurretPos(localizationSubsystem.getTurretPosition());
    }
}
