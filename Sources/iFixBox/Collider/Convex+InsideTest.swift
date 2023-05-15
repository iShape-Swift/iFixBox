//
//  Convex+InsideTest.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public extension Array where Element == FixVec {
    
    func isPointInsideConvexPolygon(point: FixVec) -> Bool {
        let p0 = self[0]
        let p1 = self[1]

        if orientation(a: p0, b: p1, c: point) < 0 {
            return false
        }

        var low = 1
        var high = count - 1

        while high - low > 1 {
            let mid = (low + high) >> 1
            let pm = self[mid]
            if orientation(a: p0, b: pm, c: point) < 0 {
                high = mid
            } else {
                low = mid
            }
        }

        return isTriangleContain(a: p0, b: self[low], c: self[high], p: point)
    }

    private func orientation(a: FixVec, b: FixVec, c: FixVec) -> FixFloat {
        (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y)
    }

    private func isTriangleContain(a: FixVec, b: FixVec, c: FixVec, p: FixVec) -> Bool {
        let s0 = orientation(a: a, b: b, c: p)
        let s1 = orientation(a: b, b: c, c: p)
        let s2 = orientation(a: c, b: a, c: p)

        return s0 >= 0 && s1 >= 0 && s2 >= 0
    }
}
