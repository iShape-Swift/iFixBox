//
//  WorldSettings.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public struct WorldSettings {

    public static let `default`: WorldSettings = .init(timeStep: 16, bodyCapacity: 1024)
    
    public let timeStep: FixFloat
    public let bodyCapacity: Int
    public let freezeMargin: FixFloat
    public let gridSpaceFactor: Int
    

    public init(timeStep: FixFloat, bodyCapacity: Int, freezeMargin: FixFloat = .unit, gridSpaceFactor: Int = 4) {
        self.timeStep = timeStep
        self.bodyCapacity = bodyCapacity
        self.freezeMargin = freezeMargin
        self.gridSpaceFactor = gridSpaceFactor
    }

}
