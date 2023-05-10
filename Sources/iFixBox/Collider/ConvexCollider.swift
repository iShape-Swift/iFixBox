//
//  ConvexCollider.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public struct ConvexCollider {

    public let center: FixVec
    public let points: [FixVec]
    public let normals: [FixVec]
    public let boundary: Boundary
    public let radius: FixFloat

    public var circleCollider: CircleCollider {
        return CircleCollider(center: center, radius: radius)
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
        self.boundary = Boundary(min: FixVec(-a, -b), max: FixVec(a, b))
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

            let nm = FixVec(-e.y, e.x).normalize
            normals[j] = nm

            let crossProduct = p1.crossProduct(p0)
            area += crossProduct

            let sp = p0 + p1
            centroid = centroid + sp * crossProduct

            p0 = p1
            j = i
        }

        area >>= 1
        let s = 6 * area

        let x = centroid.x.div(s)
        let y = centroid.y.div(s)

        self.center = FixVec(x, y)
        self.points = points
        self.normals = normals
        self.boundary = Boundary(points: points)

        var minR = Int64.max

        for i in 0..<points.count {
            let p = points[i]
            let n = normals[i]

            let v = p - center
            let r = abs(v.dotProduct(n))

            if r < minR {
                minR = r
            }
        }

        self.radius = minR
    }

    @inlinable
    public init(transform: Transform, collider: ConvexCollider) {
        let n = collider.points.count
        var points: [FixVec] = []
        var normals: [FixVec] = []
        for i in 0..<n {
            points.append(transform.convertAsPoint(collider.points[i]))
            normals.append(transform.convertAsVector(collider.normals[i]))
        }

        self.points = points
        self.normals = normals
        self.boundary = transform.convert(collider.boundary)
        self.center = transform.convertAsPoint(collider.center)
        self.radius = collider.radius
    }
    
    @inlinable
    public func isContain(_ point: FixVec) -> Bool {
        guard boundary.isContain(point: point) else {
            return false
        }

        return points.isPointInsideConvexPolygon(point: point)
    }
}
