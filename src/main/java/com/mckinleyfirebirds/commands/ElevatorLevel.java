package com.mckinleyfirebirds.commands;

public enum ElevatorLevel {
    LEVEL_ONE(0.253),
    LEVEL_TWO(0.355),
    LEVEL_THREE(0.465),
    LEVEL_FOUR(0.65),
    LEVEL_GRAB(0.4);

    private final double height;

    ElevatorLevel(double height) {
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
