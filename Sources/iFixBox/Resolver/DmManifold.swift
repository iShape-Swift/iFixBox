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
        ke = -max(a.material.bounce, b.material.bounce) - .unit
        q = (a.material.friction + b.material.friction) >> 1
        
        if contact.penetration < 0 {
            let l = -contact.penetration
            bias = l.mul(biasScale)
        } else {
            bias = 0
        }
        
        // distance between center of Mass A to contact point
        aR = contact.point - a.transform.position
        
        // distance between center of Mass B to contact point
        bR = contact.point - b.transform.position

        let ii = a.unitInertia * b.unitInertia
        
        // normal impulse
        
        aRn = aR.crossProduct(n)
        bRn = bR.crossProduct(n)

        let i = (ke << FixFloat.pentaFactionBits) / (ii * (a.mass + b.mass) + aRn.sqr * b.inertia + bRn.sqr * a.inertia)

        iVa = (i * ii.mul(b.mass)) >> FixFloat.tetraFactionBits
        iVb = (i * ii.mul(a.mass)) >> FixFloat.tetraFactionBits
        
        iWa = (i * aRn * b.inertia) >> FixFloat.tetraFactionBits
        iWb = (i * bRn * a.inertia) >> FixFloat.tetraFactionBits
        
        // tangent impulse
        
        aRt = aR.crossProduct(t)
        bRt = bR.crossProduct(t)
        
        let j = (.unit << FixFloat.pentaFactionBits) / (ii * (a.mass + b.mass) + aRt.sqr * b.inertia + bRt.sqr * a.inertia)

        jVa = (j * ii.mul(b.mass)) >> FixFloat.tetraFactionBits
        jVb = (j * ii.mul(a.mass)) >> FixFloat.tetraFactionBits
        
        jWa = (j * aRt * b.inertia) >> FixFloat.tetraFactionBits
        jWb = (j * bRt * a.inertia) >> FixFloat.tetraFactionBits
    }
    
    @inlinable
    func resolve(velA: Velocity, velB: Velocity) -> DmSolution {
        // start linear and angular velocity for A and B
        let aV1 = velA.linear
        let aW1 = velA.angular
        
        let bV1 = velB.linear
        let bW1 = velB.angular
        
        // relative velocity
        let rV1 = aV1 - bV1 + aR.crossProduct(aW1) - bR.crossProduct(bW1)
        
        let rV1proj = rV1.dotProduct(n)
        
        // only if getting closer
        guard rV1proj < 0 else {
            return .noImpact
        }

        // new linear velocity
  
        var adV = iVa.mul(rV1proj) * n
        var bdV = iVb.mul(rV1proj) * n
        
        // new angular velocity

        var adW = iWa.mul(rV1proj)
        var bdW = iWb.mul(rV1proj)
        
        // tangent vector
        let tDot = rV1.dotProduct(t)

        // ignore if it to small
        if tDot >= 1 {
            // can not be more then original vel
            let max = -rV1proj
            
            let tV1proj = tDot.clamp(min: -max, max: max).mul(q)
            
            let adVt = jVa.mul(tV1proj)
            let adWt = jWa.mul(tV1proj)

            let bdVt = jVb.mul(tV1proj)
            let bdWt = jWb.mul(tV1proj)
            
            adV = adV - adVt * t
            adW = adW - adWt
            
            bdV = bdV - bdVt * t
            bdW = bdW - bdWt
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
        let rV1 = aV1 - bV1 + aR.crossProduct(aW1) - bR.crossProduct(bW1)
        
        let rV1proj = rV1.dotProduct(n) - bias
        
        // only if getting closer
        guard rV1proj < 0 else {
            return .noImpact
        }
        
        let aV2 = aV1 + iVa.mul(rV1proj) * n
        let aW2 = aW1 + iWa.mul(rV1proj)

        let bV2 = bV1 - iVb.mul(rV1proj) * n
        let bW2 = bW1 - iWb.mul(rV1proj)
        
        return DmSolution(
            velA: Velocity(linear: aV2, angular: aW2),
            velB: Velocity(linear: bV2, angular: bW2),
            isImpact: true
        )
    }
}
