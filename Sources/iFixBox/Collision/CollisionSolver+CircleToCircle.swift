//
//  CollisionSolver+CircleToCircle.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public extension CollisionSolver {
    
    func collide(_ a: CircleCollider, _ b: CircleCollider) -> Contact {
        let ca = a.center
        let cb = b.center

        let ra = a.radius
        let rb = b.radius

        let sqrC = ca.sqrDistance(cb)

        if (ra + rb).fixSqr >= sqrC {
            let penetration = ra + rb - sqrC.fixSqrt
                
            let sqrA = ra.fixSqr
            let sqrB = rb.fixSqr

            let dv = ca - cb
            
            if sqrC >= sqrA && sqrC >= sqrB {
                let k = (sqrB - sqrA + sqrC).fixDiv(sqrC << 1)

                let p = cb + dv * k

                let nA = dv.fixNormalize
                
                return Contact(
                    point: p,
                    normal: nA,
                    penetration: -penetration,
                    status: .collide,
                    type: .vertex
                )
            } else {
                let p = sqrB > sqrA ? ca : cb
                let n = dv.fixSqrLength != 0 ? dv.fixNormalize : FixVec(x: 0, y: .unit)
                    
                return Contact(
                    point: p,
                    normal: n,
                    penetration: penetration,
                    status: .inside,
                    type: .vertex
                )
            }
        } else {
            return Contact.outside
        }
    }
    
}
