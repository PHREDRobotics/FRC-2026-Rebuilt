package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory){
        m_factory = factory;
    }

    public AutoRoutine testPathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("TestPath");
        final AutoTrajectory testPath = routine.trajectory("TestPath");

        routine.active().onTrue(
            testPath.resetOdometry()
                .andThen(testPath.cmd())
        );
        return routine;
    }
}
