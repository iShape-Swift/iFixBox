//
//  StManifold.swift
//  
//
//  Created by Nail Sharipov on 12.05.2023.
//

// https://chrishecker.com/Rigid_Body_Dynamics

import iFixFloat

struct StImpactSolution {
    
    static let noImpact = StImpactSolution(vel: .zero, isImpact: false)
    let vel: Velocity
    let isImpact: Bool
}

struct StManifold {

    let bRvw: FixVec
    
    let ke: FixFloat
    let q: FixFloat
    let bias: FixFloat
    
    let n: FixVec       // normal
    let t: FixVec       // tangent
    
    let iA: Int         // global index in bodies Array
    var vA: Int = -1    // index in vars

    let aR: FixVec

    let iVa: FixFloat
    let iWa: FixFloat

    let jVa: FixFloat
    let jWa: FixFloat
    
    @inlinable
    init(a: Body, b: Body, iA: Int, iB: Int, contact: Contact, biasScale: FixFloat) {
        self.iA = iA
        
        // Normal is always look at A * <-| * B
        n = contact.normal
        t = FixVec(n.y, -n.x)
        
        // normal impulse
        // -(1 + e) * rV1 * n / (1 / Ma + (aR * n)^2 / aI)
        
        // -(1 + e)
        ke = max(a.material.bounce, b.material.bounce) + .unit
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
        let bR = contact.point - b.transform.position
        bRvw = b.velocity.linear.negative - bR.crossProduct(b.velocity.angular)

        let aRn = aR.crossProduct(n)
        let aRt = aR.crossProduct(t)

        let i = (ke << FixFloat.cubeFactionBits) / (a.unitInertia + aRn.sqr)
        let j = (q << FixFloat.cubeFactionBits) / (a.unitInertia + aRt.sqr)
        
        iVa = -((a.unitInertia * i) >> FixFloat.cubeFactionBits)
        iWa = -((aRn * i) >> FixFloat.cubeFactionBits)

        jVa = -((a.unitInertia * j) >> FixFloat.cubeFactionBits)
        jWa = -((aRt * j) >> FixFloat.cubeFactionBits)
    }
    
    @inlinable
    func resolve(velA: Velocity) -> StImpactSolution {

        let aV1 = velA.linear
        let aW1 = velA.angular
        
        // relative velocity
        let rV1 = aV1 + aR.crossProduct(aW1) + bRvw

        let rV1dot = rV1.dotProduct(n)
        
        // only if getting closer
        guard rV1dot < 0 else {
            return .noImpact
        }

        var adV = iVa.mul(rV1dot) * n
        var adW = iWa.mul(rV1dot)
        
        // tangent vector
        let tV1Dot = rV1.dotProduct(t)
        
        // ignore if it to small
        if tV1Dot != 0 {
            // can not be more then original vel
            let min = rV1dot
            let max = -rV1dot
            
            let tV1 = tV1Dot.clamp(min: min, max: max).mul(q)
            
            let adVt = jVa.mul(tV1)
            let adWt = jWa.mul(tV1)
            
            adV = adV + adVt * t
            adW = adW + adWt
        }
        
        let aV2 = aV1 + adV
        let aW2 = aW1 + adW

        return StImpactSolution(vel: Velocity(linear: aV2, angular: aW2), isImpact: true)
    }
    
    @inlinable
    func resolveBias(velA: Velocity) -> StImpactSolution {

        let aV1 = velA.linear
        let aW1 = velA.angular
        
        // relative velocity
        let rV1 = aV1 + aR.crossProduct(aW1) + bRvw

        let rV1dot = rV1.dotProduct(n) - bias
        
        // only if getting closer
        guard rV1dot < 0 else {
            return .noImpact
        }

        let aV2 = aV1 + iVa.mul(rV1dot) * n
        let aW2 = aW1 + iWa.mul(rV1dot)

        return StImpactSolution(vel: Velocity(linear: aV2, angular: aW2), isImpact: true)
    }
    
}
