//
//  WorldSettings.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public enum SolverPrecision {
    case rude
    case normal
    case accurate
}

public struct WorldSettings {

    public static let `default`: WorldSettings = .init(solverPrecision: .normal, bodyCapacity: 1024)
    
    public let posTimeStep: FixFloat
    public let velocityIterations: Int
    public let positionIterations: Int
    public let bodyCapacity: Int
    public let freezeMargin: FixFloat
    public let gridSpaceFactor: Int
    

    public init(timeStep: FixFloat = 16, velocityIterations: Int = 8, positionIterations: Int = 4, bodyCapacity: Int, freezeMargin: FixFloat = .unit, gridSpaceFactor: Int = 4) {
        self.velocityIterations = velocityIterations
        self.positionIterations = positionIterations
        self.bodyCapacity = bodyCapacity
        self.freezeMargin = freezeMargin
        self.gridSpaceFactor = gridSpaceFactor
        self.posTimeStep = FixFloat(16).div(FixFloat(positionIterations))
    }
    
    public init(solverPrecision: SolverPrecision = .normal, bodyCapacity: Int, freezeMargin: FixFloat = .unit, gridSpaceFactor: Int = 4) {
        self.bodyCapacity = bodyCapacity
        self.freezeMargin = freezeMargin
        self.gridSpaceFactor = gridSpaceFactor

        switch solverPrecision {
        case .rude:
            self.velocityIterations = 4
            self.positionIterations = 2
        case .normal:
            self.velocityIterations = 8
            self.positionIterations = 4
        case .accurate:
            self.velocityIterations = 16
            self.positionIterations = 8
        }
        
        self.posTimeStep = 16 / FixFloat(positionIterations)
    }

}
