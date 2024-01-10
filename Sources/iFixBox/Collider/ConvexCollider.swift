//
//  ConvexCollider.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iShape
import iFixFloat

public struct ConvexCollider {

    public let center: FixVec
    public let points: [FixVec]
    public let normals: [FixVec]
    public let boundary: FixBnd
    public let radius: FixFloat

    public var circleCollider: CircleCollider {
        CircleCollider(center: center, radius: radius)
    }

    public init(size: Size) {
        let a = (size.width + 1) >> 1
        let b = (size.height + 1) >> 1
        let points: [FixVec] = [
            FixVec(-a, -b),
            FixVec(-a, b),
            FixVec(a, b),
            FixVec(a, -b)
        ]

        let normals: [FixVec] = [
            FixVec(-.unit, 0),
            FixVec(0, .unit),
            FixVec(.unit, 0),
            FixVec(0, -.unit)
        ]

        self.center = FixVec(0, 0)
        self.points = points
        self.normals = normals
        self.boundary = FixBnd(min: FixVec(-a, -b), max: FixVec(a, b))
        self.radius = min(a, b)
    }

    public init(points: [FixVec]) {
        assert(points.count >= 3, "At least 3 points are required")

        var normals: [FixVec] = Array(repeating: FixVec.zero, count: points.count)

        var centroid = FixVec.zero
        var area: Int64 = 0

        var j = points.count - 1
        var p0 = points[j]

        for i in 0..<points.count {
            let p1 = points[i]
            let e = p1 - p0

            let nm = FixVec(-e.y, e.x).fixNormalize
            normals[j] = nm

            let crossProduct = p1.fixCrossProduct(p0)
            area += crossProduct

            let sp = p0 + p1
            centroid = centroid + sp * crossProduct

            p0 = p1
            j = i
        }

        area >>= 1
        let s = 6 * area

        let x = centroid.x.fixDiv(s)
        let y = centroid.y.fixDiv(s)

        self.center = FixVec(x, y)
        self.points = points
        self.normals = normals
        self.boundary = FixBnd(points: points)

        var minR = Int64.max

        for i in 0..<points.count {
            let p = points[i]
            let n = normals[i]

            let v = p - center
            let r = abs(v.fixDotProduct(n))

            if r < minR {
                minR = r
            }
        }

        self.radius = minR
    }

    @inlinable
    public init(transform: Transform, collider: ConvexCollider) {
        let n = collider.points.count
        var points = [FixVec](repeating: .zero, count: n)
        var normals = [FixVec](repeating: .zero, count: n)
        
        var minX = FixFloat.max
        var maxX = FixFloat.min
        var minY = FixFloat.max
        var maxY = FixFloat.min
        
        for i in 0..<n {
            let p = transform.convertAsPoint(collider.points[i])
            points[i] = p
            
            normals[i] = transform.convertAsVector(collider.normals[i])
            
            minX = Swift.min(minX, p.x)
            maxX = Swift.max(maxX, p.x)
            minY = Swift.min(minY, p.y)
            maxY = Swift.max(maxY, p.y)
        }

        self.points = points
        self.normals = normals
        self.boundary = FixBnd(min: FixVec(minX, minY), max: FixVec(maxX, maxY))
        self.center = transform.convertAsPoint(collider.center)
        self.radius = collider.radius
    }
}
