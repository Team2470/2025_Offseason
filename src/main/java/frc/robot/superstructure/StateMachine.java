package frc.robot.superstructure;

// Simple state machine runner
public class StateMachine {
    private State current;
    private long enterTimestampMs = 0;

    public void setState(State s) {
        if (current != null) current.onExit();
        current = s;
        if (current != null) {
            current.onEnter();
            enterTimestampMs = System.currentTimeMillis();
        }
    }

    public void periodic() {
        if (current == null){ 
            return;
        }
        current.periodic();
        if (current.isFinished()) {
            State nxt = current.nextState();
            setState(nxt);
        }
    }

    public long timeInStateMs() {
        return System.currentTimeMillis() - enterTimestampMs;
    }

    public State getCurrent() {
        return current; 
    }
}
