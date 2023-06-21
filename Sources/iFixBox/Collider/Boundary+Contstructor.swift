//
//  Boundary.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public extension Boundary {
    
    init(size: Size) {
        let half = size.half
        self.init(min: half.negative, max: half)
    }

    init(radius: FixFloat, delta: FixVec) {
        self.init(min: FixVec(-radius, -radius) + delta, max: FixVec(radius, radius) + delta)
    }

    init(size: Size, transform: Transform) {
        let a = (size.width + 1) >> 1
        let b = (size.height + 1) >> 1
        let p0 = transform.convertAsPoint(FixVec(-a, -b))
        let p1 = transform.convertAsPoint(FixVec(-a, b))
        let p2 = transform.convertAsPoint(FixVec(a, b))
        let p3 = transform.convertAsPoint(FixVec(a, -b))

        let minX = Swift.min(p0.x, p1.x, p2.x, p3.x)
        let maxX = Swift.max(p0.x, p1.x, p2.x, p3.x)
        let minY = Swift.min(p0.y, p1.y, p2.y, p3.y)
        let maxY = Swift.max(p0.y, p1.y, p2.y, p3.y)

        self.init(min: FixVec(minX, minY), max: FixVec(maxX, maxY))
    }
    
    func translate(delta: FixVec) -> Boundary {
        Boundary(min: min + delta, max: max + delta)
    }
}
