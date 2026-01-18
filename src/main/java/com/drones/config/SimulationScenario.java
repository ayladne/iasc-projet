package com.drones.config;

/**
 * Scénarios de simulation prédéfinis pour les tests
 */
public enum SimulationScenario {
    
    NO_ANOMALIES(
        "Pas d'anomalies",
        0.0, // probabilité d'apparition
        0.9, // taux de décomposition
        0.05 // diffusion
    ),
    
    SPARSE_ANOMALIES(
        "Anomalies sporadiques",
        0.02, // probabilité d'apparition
        0.93, // taux de décomposition
        0.08 // diffusion
    ),
    
    NORMAL_SCENARIO(
        "Scénario normal",
        0.05, // probabilité d'apparition
        0.95, // taux de décomposition
        0.10 // diffusion
    ),
    
    HEAVY_POLLUTION(
        "Pollution intense",
        0.15, // probabilité d'apparition
        0.92, // taux de décomposition
        0.15 // diffusion
    ),
    
    RAPIDLY_SPREADING(
        "Propagation rapide",
        0.08, // probabilité d'apparition
        0.90, // taux de décomposition
        0.20 // diffusion
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
        // Dans un vrai projet, on utiliserait l'injection de dépendances
        // Pour maintenant, ceci est un placeholder pour l'ajustement dynamique des paramètres
    }
}
