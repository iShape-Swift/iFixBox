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

    public var isZero: Bool {
        return linear == .zero && angular == 0
    }

    public init(linear: FixVec, angular: FixFloat = 0) {
        self.linear = linear
        self.angular = angular
    }
}
