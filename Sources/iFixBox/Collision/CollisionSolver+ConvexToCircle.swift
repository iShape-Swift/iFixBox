//
//  CollisionSolver+ConvexToCircle.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public extension CollisionSolver {
    
    func collide(_ circle: CircleCollider, _ convex: ConvexCollider) -> Contact {
        var normalIndex = 0
        var separation: Int64 = Int64.min

        let r: Int64 = circle.radius + 10 // TODO: review this constant

        for i in 0..<convex.points.count {
            let d = circle.center - convex.points[i]
            let s = convex.normals[i].dotProduct(d)

            if s > r {
                return Contact.outside
            }

            if s > separation {
                separation = s
                normalIndex = i
            }
        }

        let vertIndex1 = normalIndex
        let vertIndex2 = (vertIndex1 + 1) % convex.points.count
        let v1 = convex.points[vertIndex1]
        let v2 = convex.points[vertIndex2]
        let n1 = convex.normals[vertIndex1]

        let delta = separation - circle.radius
        
        if separation < 0 {
            return Contact(
                point: circle.center,
                normal: n1,
                penetration: delta,
                status: .inside,
                type: .vertex
            )
        }

        let sqrRadius = circle.radius.sqr

        let u1 = (circle.center - v1).dotProduct(v2 - v1)

        if u1 <= 0 {
            if circle.center.sqrDistance(v1) > sqrRadius {
                return Contact.outside
            }

            let nB = (circle.center - v1).normalize
            return Contact(point: v1, normal: nB, penetration: delta, status: .collide, type: .vertex)
        }

        let u2 = (circle.center - v2).dotProduct(v1 - v2)

        if u2 <= 0 {
            if circle.center.sqrDistance(v2) > sqrRadius {
                return Contact.outside
            }

            let nB = (circle.center - v2).normalize
            return Contact(point: v2, normal: nB, penetration: delta, status: .collide, type: .vertex)
        }

        let faceCenter = v1.middle(v2)

        let sc = (circle.center - faceCenter).dotProduct(n1)
        if sc > circle.radius {
            return Contact.outside
        }

        let dc = (circle.center - v2).dotProduct(n1)
        let m = circle.center - dc * n1

        return Contact(point: m, normal: n1, penetration: delta, status: .collide, type: .vertex)
    }
}
