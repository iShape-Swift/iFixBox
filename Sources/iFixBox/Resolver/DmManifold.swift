//
//  DmSolution.swift
//  
//
//  Created by Nail Sharipov on 12.05.2023.
//

// https://chrishecker.com/Rigid_Body_Dynamics

import iFixFloat

struct DmSolution {
    
    static let noImpact = DmSolution(velA: .zero, velB: .zero, isImpact: false)

    let velA: Velocity
    let velB: Velocity
    let isImpact: Bool
}

struct DmManifold {
    let ke: FixFloat
    let q: FixFloat
    let bias: FixFloat
    
    let n: FixVec
    let t: FixVec
    
    let iA: Int         // global index in bodies Array
    let iB: Int         // global index in bodies Array
    
    var vA: Int = -1    // index in vars
    var vB: Int = -1    // index in vars
    
    let aR: FixVec
    let bR: FixVec
    
    let aRn: FixFloat
    let bRn: FixFloat
    
    let aRt: FixFloat
    let bRt: FixFloat
    
    let iVa: FixFloat
    let iVb: FixFloat

    let iWa: FixFloat
    let iWb: FixFloat
    
    let jVa: FixFloat
    let jVb: FixFloat

    let jWa: FixFloat
    let jWb: FixFloat
    
    @inlinable
    init(a: Body, b: Body, iA: Int, iB: Int, contact: Contact, biasScale: FixFloat) {

        self.iA = iA
        self.iB = iB
        
        // Normal is always look at A * <-| * B
        n = contact.normal
        t = FixVec(n.y, -n.x)
        
        // -(1 + e)
        ke = max(a.material.bounce, b.material.bounce) + .unit

        q = (a.material.friction + b.material.friction) >> 1
        
        if contact.penetration < 0 {
            let l = -contact.penetration
            bias = l.fixMul(biasScale)
        } else {
            bias = 0
        }
        
        // distance between center of Mass A to contact point
        aR = contact.point - a.transform.position
        
        // distance between center of Mass B to contact point
        bR = contact.point - b.transform.position

        let ii = a.unitInertia * b.unitInertia
        
        // normal impulse
        
        aRn = aR.fixCrossProduct(n)
        bRn = bR.fixCrossProduct(n)

        let i = (ke << FixFloat.pentaFactionBits) / (ii * (a.mass + b.mass) + aRn.fixSqr * b.inertia + bRn.fixSqr * a.inertia)

        iVa = -((i * ii.fixMul(b.mass)) >> FixFloat.tetraFactionBits)
        iVb = -((i * ii.fixMul(a.mass)) >> FixFloat.tetraFactionBits)
        
        iWa = -((i * aRn * b.inertia) >> FixFloat.tetraFactionBits)
        iWb = -((i * bRn * a.inertia) >> FixFloat.tetraFactionBits)
        
        // tangent impulse
        
        aRt = aR.fixCrossProduct(t)
        bRt = bR.fixCrossProduct(t)
        
        let j = (.unit << FixFloat.pentaFactionBits) / (ii * (a.mass + b.mass) + aRt.fixSqr * b.inertia + bRt.fixSqr * a.inertia)

        jVa = -((j * ii.fixMul(b.mass)) >> FixFloat.tetraFactionBits)
        jVb = -((j * ii.fixMul(a.mass)) >> FixFloat.tetraFactionBits)
        
        jWa = -((j * aRt * b.inertia) >> FixFloat.tetraFactionBits)
        jWb = -((j * bRt * a.inertia) >> FixFloat.tetraFactionBits)
    }
    
    @inlinable
    func resolve(velA: Velocity, velB: Velocity) -> DmSolution {
        // start linear and angular velocity for A and B
        let aV1 = velA.linear
        let aW1 = velA.angular
        
        let bV1 = velB.linear
        let bW1 = velB.angular
        
        // relative velocity
        let rV1 = aV1 - bV1 + aR.fixCrossProduct(aW1) - bR.fixCrossProduct(bW1)
        
        let rV1dot = rV1.fixDotProduct(n)
        
        // only if getting closer
        guard rV1dot < 0 else {
            return .noImpact
        }

        // new linear velocity
  
        var adV = n.fixMul(iVa.fixMul(rV1dot))
        var bdV = n.fixMul(iVb.fixMul(rV1dot))
        
        // new angular velocity

        var adW = iWa.fixMul(rV1dot)
        var bdW = iWb.fixMul(rV1dot)
        
        // tangent vector
        let tV1Dot = rV1.fixDotProduct(t)

        // ignore if it to small
        if tV1Dot != 0 {
            // can not be more then original vel
            let min = rV1dot
            let max = -rV1dot
            
            let tV1 = tV1Dot.clamp(min: min, max: max).fixMul(q)
            
            let adVt = jVa.fixMul(tV1)
            let adWt = jWa.fixMul(tV1)

            let bdVt = jVb.fixMul(tV1)
            let bdWt = jWb.fixMul(tV1)
            
            adV = adV + t.fixMul(adVt)
            adW = adW + adWt
            
            bdV = bdV + t.fixMul(bdVt)
            bdW = bdW + bdWt
        }

        let aV2 = aV1 + adV
        let aW2 = aW1 + adW

        let bV2 = bV1 - bdV
        let bW2 = bW1 - bdW
        
        return DmSolution(
            velA: Velocity(linear: aV2, angular: aW2),
            velB: Velocity(linear: bV2, angular: bW2),
            isImpact: true
        )
    }
    
    @inlinable
    func resolveBias(velA: Velocity, velB: Velocity) -> DmSolution {
        // start linear and angular velocity for A and B
        let aV1 = velA.linear
        let aW1 = velA.angular
        
        let bV1 = velB.linear
        let bW1 = velB.angular
        
        // relative velocity
        let rV1 = aV1 - bV1 + aR.fixCrossProduct(aW1) - bR.fixCrossProduct(bW1)
        
        let rV1dot = rV1.fixDotProduct(n) - bias
        
        // only if getting closer
        guard rV1dot < 0 else {
            return .noImpact
        }
        
        let aV2 = aV1 + n.fixMul(iVa.fixMul(rV1dot))
        let aW2 = aW1 + iWa.fixMul(rV1dot)

        let bV2 = bV1 - n.fixMul(iVb.fixMul(rV1dot))
        let bW2 = bW1 - iWb.fixMul(rV1dot)
        
        return DmSolution(
            velA: Velocity(linear: aV2, angular: aW2),
            velB: Velocity(linear: bV2, angular: bW2),
            isImpact: true
        )
    }
}
