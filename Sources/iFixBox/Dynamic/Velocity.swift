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

    @inlinable
    public var isZero: Bool {
        linear == .zero && angular == 0
    }

    @inlinable
    public init(linear: FixVec, angular: FixFloat = 0) {
        self.linear = linear
        self.angular = angular
    }

    @inlinable
    public func apply(timeStep: Int64, acceleration: Acceleration) -> Velocity {
        let dA = acceleration.linear.fixMul(timeStep)
        let dWa = acceleration.angular * timeStep
        return Velocity(linear: linear + dA, angular: angular + dWa)
    }
    
    
    @inlinable
    static func +(left: Velocity, right: Velocity) -> Velocity {
        Velocity(linear: left.linear + right.linear, angular: left.angular + right.angular)
    }

    @inlinable
    static func -(left: Velocity, right: Velocity) -> Velocity {
        Velocity(linear: left.linear - right.linear, angular: left.angular - right.angular)
    }
    
}
