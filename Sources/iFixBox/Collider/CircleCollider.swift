//
//  CircleCollider.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat


public struct CircleCollider {

    public let center: FixVec
    public let radius: FixFloat

    public init(center: FixVec, radius: FixFloat) {
        self.center = center
        self.radius = radius
    }

}
