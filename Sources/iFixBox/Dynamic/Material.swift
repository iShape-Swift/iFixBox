//
//  Material.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public struct Material {
    
    public static let ordinary: Material = Material(bounce: 256, friction: 512, density: .unit, airLinearFriction: .unit, airAngularFriction: .unit)
    
    public let bounce: FixFloat
    public let friction: FixFloat
    public let density: FixFloat
    public let airLinearFriction: FixFloat
    public let airAngularFriction: FixFloat

    public init(bounce: FixFloat, friction: FixFloat, density: FixFloat, airLinearFriction: FixFloat, airAngularFriction: FixFloat) {
        self.bounce = bounce
        self.friction = friction
        self.density = density
        self.airLinearFriction = airLinearFriction
        self.airAngularFriction = airAngularFriction
    }
}
