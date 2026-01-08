package com.drones.config;

/**
 * Pre-defined simulation scenarios for testing
 */
public enum SimulationScenario {
    
    NO_ANOMALIES(
        "Pas d'anomalies",
        0.0,      // spawn probability
        0.9,      // decay rate
        0.05      // diffusion
    ),
    
    SPARSE_ANOMALIES(
        "Anomalies sporadiques",
        0.02,     // spawn probability
        0.93,     // decay rate
        0.08      // diffusion
    ),
    
    NORMAL_SCENARIO(
        "Scénario normal",
        0.05,     // spawn probability
        0.95,     // decay rate
        0.10      // diffusion
    ),
    
    HEAVY_POLLUTION(
        "Pollution intense",
        0.15,     // spawn probability
        0.92,     // decay rate
        0.15      // diffusion
    ),
    
    RAPIDLY_SPREADING(
        "Propagation rapide",
        0.08,     // spawn probability
        0.90,     // decay rate
        0.20      // diffusion
    );
    
    public final String name;
    public final double spawnProbability;
    public final double decayRate;
    public final double diffusionFactor;
    
    SimulationScenario(String name, double spawn, double decay, double diffusion) {
        this.name = name;
        this.spawnProbability = spawn;
        this.decayRate = decay;
        this.diffusionFactor = diffusion;
    }
    
    public static void applyScenario(SimulationScenario scenario) {
        // In a real project, we'd use dependency injection
        // For now, this is a placeholder for dynamic parameter adjustment
    }
}
