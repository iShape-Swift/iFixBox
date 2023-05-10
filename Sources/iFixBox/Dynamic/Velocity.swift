//
//  Velocity.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public struct Velocity {
    
    public static let zero = Velocity(linear: .zero)

    public let linear: FixVec
    public let angular: FixFloat

    public var isZero: Bool {
        return linear == .zero && angular == 0
    }

    public init(linear: FixVec, angular: FixFloat = 0) {
        self.linear = linear
        self.angular = angular
    }

    public func apply(timeStep: Int64, acceleration: Acceleration) -> Velocity {
        let dA = acceleration.linear * timeStep
        let dWa = acceleration.angular * timeStep
        return Velocity(linear: linear + dA, angular: angular + dWa)
    }
}
