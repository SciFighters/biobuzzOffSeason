package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.commands.DriveCmds;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;

/**
 * Test autonomous using Pinpoint field-oriented navigation.
 * Drives to center, spins 360, shuttles 50cm back and forth twice.
 */
@Autonomous(name = "TestAuto PedroPath", group = "Auto")
public class TestAuto extends CommandOpMode {

    private DriveSub driveSub;

    @Override
    public void initialize() {
        HardwareConfig hm = new HardwareConfig();
        hm.init(hardwareMap);
        driveSub = new DriveSub(hm);
        driveSub.setActiveGuard(() -> true);

        // Reset, drive to center, spin 360, then 50cm forward/back twice.
        schedule(DriveCmds.testAutoSequence(driveSub));
    }
}
