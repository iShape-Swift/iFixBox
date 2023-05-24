//
//  Boundary.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

/*
import iFixFloat

public struct Boundary {

    public static let zero = Boundary(min: .zero, max: .zero)
    
    public let min: FixVec
    public let max: FixVec
    
    @inlinable
    public init(min: FixVec, max: FixVec) {
        self.min = min
        self.max = max
    }

    @inlinable
    public init(radius: FixFloat) {
        self.min = FixVec(-radius, -radius)
        self.max = FixVec(radius, radius)
    }
    
    public init(size: Size) {
        let half = size.half
        min = half.negative
        max = half
    }
    
    @inlinable
    public init(points: [FixVec]) {
        let p0 = points[0]
        var minX = p0.x
        var maxX = p0.x
        var minY = p0.y
        var maxY = p0.y

        for i in 1..<points.count {
            let p = points[i]

            minX = Swift.min(minX, p.x)
            maxX = Swift.max(maxX, p.x)
            minY = Swift.min(minY, p.y)
            maxY = Swift.max(maxY, p.y)
        }
        
        self.min = FixVec(minX, minY)
        self.max = FixVec(maxX, maxY)
    }

    public func translate(delta: FixVec) -> Boundary {
        Boundary(min: min + delta, max: max + delta)
    }
    
    @inlinable
    public func union(_ box: Boundary) -> Boundary {
        let minX = Swift.min(min.x, box.min.x)
        let minY = Swift.min(min.y, box.min.y)
        let maxX = Swift.max(max.x, box.max.x)
        let maxY = Swift.max(max.y, box.max.y)
        
        return Boundary(min: FixVec(minX, minY), max: FixVec(maxX, maxY))
    }
    
    @inlinable
    public func isCollide(_ box: Boundary) -> Bool {
        // Check if the bounding boxes intersect in any dimension
        if max.x < box.min.x || min.x > box.max.x {
            return false
        }
        if max.y < box.min.y || min.y > box.max.y {
            return false
        }
        return true
    }
    
    @inlinable
    public func isContain(point p: FixVec) -> Bool {
        min.x <= p.x && p.x <= max.x && min.y <= p.y && p.y <= max.y
    }

    @inlinable
    public func isCollideCircle(center: FixVec, radius: FixFloat) -> Bool {
        let cx = Swift.max(min.x, Swift.min(center.x, max.x))
        let cy = Swift.max(min.y, Swift.min(center.y, max.y))

        let sqrDist = FixVec(cx, cy).sqrDistance(center)

        return sqrDist <= radius.sqr
    }

}
*/

extension Boundary {
    
    init(size: Size) {
        let half = size.half
        self.init(min: half.negative, max: half)
    }
    
    func translate(delta: FixVec) -> Boundary {
        Boundary(min: min + delta, max: max + delta)
    }
    
}
