//
//  CollisionSolver+CircleToCircle.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

extension CollisionSolver {
    
    func collide(_ a: CircleCollider, _ b: CircleCollider) -> Contact {
        let ca = a.center
        let cb = b.center

        let ra = a.radius
        let rb = b.radius

        let sqrC = ca.sqrDistance(cb)

        if (ra + rb).sqr >= sqrC {
            let penetration = ra + rb - sqrC.sqrt
                
            let sqrA = ra.sqr
            let sqrB = rb.sqr

            let dv = ca - cb
            
            if sqrC >= sqrA && sqrC >= sqrB {
                let k = (sqrB - sqrA + sqrC).div(sqrC << 1)

                let p = cb + dv * k

                let nA = dv.normalize
                
                return Contact(
                    point: p,
                    normal: nA,
                    penetration: -penetration,
                    count: 1,
                    type: .collide
                )
            } else {
                let p = sqrB > sqrA ? ca : cb
                let n = dv.sqrLength != 0 ? dv.normalize : FixVec(x: 0, y: .unit)
                    
                return Contact(
                    point: p,
                    normal: n,
                    penetration: penetration,
                    count: 1,
                    type: .inside
                )
            }
        } else {
            return Contact.outside
        }
    }
    
}
