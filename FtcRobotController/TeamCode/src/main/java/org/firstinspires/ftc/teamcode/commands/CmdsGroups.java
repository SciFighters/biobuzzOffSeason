package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subSystems.BoxSub;
import org.firstinspires.ftc.teamcode.subSystems.DriveSub;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSub;
import org.firstinspires.ftc.teamcode.subSystems.LiftSub;

public class CmdsGroups {

    public static class Home extends SequentialCommandGroup {
        public Home(LiftSub liftSub, BoxSub boxSub) {
            addCommands(
                    new LiftCmds.LiftGoToHeight(liftSub, 0),
                    new BoxCmds.Reset(boxSub)
            );
        }
    }

    public static class ResetAll extends SequentialCommandGroup {
        public ResetAll(DriveSub drive, LiftSub liftSub, BoxSub boxSub) {
            addCommands(
//                    new DriveCmds.ResetPose(drive),//error
                    new LiftCmds.LiftGoToHeight(liftSub, 0),
                    new BoxCmds.Reset(boxSub)
            );
        }
    }

    public static class Pickup extends SequentialCommandGroup {
        public Pickup(LiftSub liftSub, BoxSub boxSub, IntakeSub intakeSub) {
            addCommands(
                    new Home(liftSub, boxSub),
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

    // full cycle of pickup, lift to target height, and reset to home position
    public static class LightWorkNoReaction extends SequentialCommandGroup {
        public LightWorkNoReaction(DriveSub drive, LiftSub liftSub, BoxSub boxSub, IntakeSub intakeSub) {
            addCommands(
                    new Pickup(liftSub, boxSub, intakeSub),
                    new TargetHeight(liftSub, boxSub),
                    new Home(liftSub, boxSub)
            );
        }
    }
}
