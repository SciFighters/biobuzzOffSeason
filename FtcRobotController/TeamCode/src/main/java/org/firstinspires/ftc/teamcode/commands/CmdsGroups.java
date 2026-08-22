package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subSystems.BoxSub;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSub;
import org.firstinspires.ftc.teamcode.subSystems.LiftSub;

public class CmdsGroups {

    public static class HomeReset extends SequentialCommandGroup {
        public HomeReset(LiftSub liftSub, BoxSub boxSub) {
            addCommands(
                    new LiftCmds.LiftGoToHeight(liftSub, 0),
                    new BoxCmds.Reset(boxSub)
            );
        }
    }

    public static class ResetAll extends SequentialCommandGroup {
        public ResetAll(DriveSub drive, LiftSub liftSub, BoxSub boxSub) {
            addCommands(
                    new DriveCmds.ResetPose(drive),
                    new LiftCmds.LiftGoToHeight(liftSub, 0),
                    new BoxCmds.Reset(boxSub)
            );
        }
    }

    public static class Pickup extends SequentialCommandGroup {
        public Pickup(LiftSub liftSub, BoxSub boxSub, IntakeSub intakeSub) {
            addCommands(
                    new HomeReset(liftSub, boxSub),
                    new IntakeCmds.Intake(intakeSub)
            );
        }
    }

    public static class TargetHeight extends SequentialCommandGroup {
        public TargetHeight(LiftSub liftSub, BoxSub boxSub) {
            addCommands(
                    new LiftCmds.LiftGoToHeight(liftSub, 0.6),
                    new BoxCmds.Discharge(boxSub)
            );
        }
    }

    public static class Cycle extends SequentialCommandGroup {
        public Cycle(DriveSub drive, LiftSub liftSub, BoxSub boxSub, IntakeSub intakeSub) {
            addCommands(
                    new Pickup(liftSub, boxSub, intakeSub),
                    new TargetHeight(liftSub, boxSub),
                    new HomeReset(liftSub, boxSub)
            );
        }
    }
}
