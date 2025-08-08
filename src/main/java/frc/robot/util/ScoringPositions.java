package frc.robot.util;


public enum ScoringPositions {
    
    LoadingPosition,
    Transition,
    L1Coral(Type.Coral),
    L2Coral(Type.Coral),
    L3Coral(Type.Coral),
    L4Coral(Type.Coral),
    L2Algae(Type.Algae),
    L3Algae(Type.Algae);

    public enum Type {
        Coral,
        Algae
    }

    private Type type;

    private ScoringPositions() {
        this.type = Type.Coral;
    }

    private ScoringPositions(Type type) {
        this.type = type;
    }

    public Type getType() {
        return type;
    }
}

