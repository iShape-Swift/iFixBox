//
//  WorldSettings.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public enum SolverPrecision {
    case low
    case moderate
    case normal
    case high
    case ultra
}

public struct WorldSettings {

    public static let `default`: WorldSettings = .init(solverPrecision: .normal, bodyCapacity: 1024)
    
    public let posTimeStep: FixFloat
    public let velocityIterations: Int
    public let positionIterations: Int
    public let bodyCapacity: Int
    public let freezeMargin: FixFloat
    public let gridSpaceFactor: Int
    public let stopVel: FixFloat = 64
    public let biasImpact: Int
   

    public init(timeStep: FixFloat = 16, velocityIterations: Int = 8, positionIterations: Int = 4, bodyCapacity: Int, freezeMargin: FixFloat = .unit, gridSpaceFactor: Int = 4, biasImpact: Int = 8) {
        self.velocityIterations = velocityIterations
        self.positionIterations = positionIterations
        self.bodyCapacity = bodyCapacity
        self.freezeMargin = freezeMargin
        self.gridSpaceFactor = gridSpaceFactor
        self.posTimeStep = FixFloat(16).div(FixFloat(positionIterations))
        self.biasImpact = biasImpact
    }
    
    public init(solverPrecision: SolverPrecision = .normal, bodyCapacity: Int, freezeMargin: FixFloat = .unit, gridSpaceFactor: Int = 4, biasImpact: Int = 8) {
        self.bodyCapacity = bodyCapacity
        self.freezeMargin = freezeMargin
        self.gridSpaceFactor = gridSpaceFactor

        switch solverPrecision {
        case .low:
            self.velocityIterations = 8
            self.positionIterations = 2
        case .moderate:
            self.velocityIterations = 8
            self.positionIterations = 4
        case .normal:
            self.velocityIterations = 16
            self.positionIterations = 4
        case .high:
            self.velocityIterations = 16
            self.positionIterations = 8
        case .ultra:
            self.velocityIterations = 32
            self.positionIterations = 8
        }
        
        self.posTimeStep = FixFloat(16 / positionIterations)
        self.biasImpact = biasImpact
    }

}
