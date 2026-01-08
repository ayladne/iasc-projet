package com.drones.model;

public enum DroneState {
    ACTIVE("Actif", "green"),
    MEASURING("Mesure", "blue"),
    RETURNING("Retour base", "orange"),
    CHARGING("Recharge", "red");
    
    private final String label;
    private final String color;
    
    DroneState(String label, String color) {
        this.label = label;
        this.color = color;
    }
    
    public String getLabel() { return label; }
    public String getColor() { return color; }
}
