package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class ActionOpMode extends CommandOpMode {
    public final FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dash.getTelemetry();
    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry(dashTelemetry,telemetry);

}
