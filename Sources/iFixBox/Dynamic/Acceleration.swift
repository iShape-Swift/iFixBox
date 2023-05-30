//
//  Acceleration.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public struct Acceleration {
    
    public static let zero = Acceleration(linear: .zero)

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
}
