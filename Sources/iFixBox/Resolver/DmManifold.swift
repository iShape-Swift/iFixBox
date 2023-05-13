//
//  DmSolution.swift
//  
//
//  Created by Nail Sharipov on 12.05.2023.
//

import iFixFloat

struct DmSolution {
    
    static let noImpact = DmSolution(velA: .zero, velB: .zero, isImpact: false)

    let velA: Velocity
    let velB: Velocity
    let isImpact: Bool
}

struct DmManifold {
    
    let invMassA: FixFloat
    let invInerA: FixDouble
    
    let invMassB: FixFloat
    let invInerB: FixDouble
    
    let ke: FixFloat
    let q: FixFloat
    let bias: FixFloat
    let n: FixVec
    
    let iA: Int         // global index in bodies Array
    let iB: Int         // global index in bodies Array
    
    var vA: Int = -1    // index in vars
    var vB: Int = -1    // index in vars
    
    let aR: FixVec
    let bR: FixVec
    let aRn: FixFloat
    let bRn: FixFloat
    
    let aRf: FixFloat
    let bRf: FixFloat
    
    let iDen: FixFloat
    let jDen: FixFloat
    
    init(a: Body, b: Body, iA: Int, iB: Int, contact: Contact, iTimeStep: FixFloat) {
        invMassA = a.invMass
        invInerA = a.invInertia
        
        invMassB = b.invMass
        invInerB = b.invInertia
        
        self.iA = iA
        self.iB = iB
        
        // Normal is always look at A * <-| * B
        n = contact.normal
        
        // -(1 + e)
        ke = -max(a.material.bounce, b.material.bounce) - .unit
        q = (a.material.friction + b.material.friction) >> 1
        
        if contact.penetration < 0 {
            let l = -contact.penetration
            let maxBias = l.mul(iTimeStep) + 1
            bias = min(.half, maxBias)
        } else {
            bias = 0
        }
        
        // distance between center of Mass A to contact point
        aR = contact.point - a.transform.position
        
        // distance between center of Mass B to contact point
        bR = contact.point - b.transform.position
        
        aRn = aR.crossProduct(n)
        bRn = bR.crossProduct(n)
        
        iDen = a.invMass + b.invMass + aRn.sqr.mul(fixDouble: a.invInertia) + bRn.sqr.mul(fixDouble: b.invInertia)
        
        let t = FixVec(n.y, -n.x)
        
        aRf = aR.crossProduct(t)
        bRf = bR.crossProduct(t)
        
        jDen = a.invMass + b.invMass + aRf.sqr.mul(fixDouble: a.invInertia) + bRf.sqr.mul(fixDouble: b.invInertia)
    }
    
    func resolve(varA: VarBody, varB: VarBody) -> DmSolution {
        // start linear and angular velocity for A and B
        let aV1 = varA.velocity.linear
        let aW1 = varA.velocity.angular
        
        let bV1 = varB.velocity.linear
        let bW1 = varB.velocity.angular
        
        // relative velocity
        let rV1 = aV1 - bV1 + aR.crossProduct(aW1) - bR.crossProduct(bW1)
        
        var rV1proj = rV1.dotProduct(n)
        
        // only if getting closer
        guard rV1proj < bias else {
            return .noImpact
        }
        
        rV1proj = min(rV1proj, -bias)
        
        // normal impulse
        // -(1 + e) * rV1 * n / (1 / Ma + 1 / Mb + (aR * t)^2 / aI)
        
        let iNum = rV1proj.mul(ke)
        let i = iNum.div(iDen)
        
        // new linear velocity
        var aV2 = aV1 + i.mul(invMassA) * n
        var bV2 = bV1 - i.mul(invMassB) * n
        
        // new angular velocity
        var aW2 = aW1 + aRn.mul(i).mul(fixDouble: invInerA)
        var bW2 = bW1 - bRn.mul(i).mul(fixDouble: invInerB)
        
        // tangent vector
        // leaving only the component that is parallel to the contact surface
        let f = rV1 - n * rV1proj
        let sqrF = f.sqrLength
        
        // ignore if it to small
        if sqrF >= 1 {
            let t = FixVec(n.y, -n.x)
            
            let jNum = -rV1.dotProduct(t)
            
            var j = jNum.div(jDen)
            
            // can not be more then original impulse
            let maxFi = i.mul(q)
            j = j.clamp(min: -maxFi, max: maxFi)
            
            // new linear velocity
            aV2 = aV2 + j.mul(invMassA) * t
            bV2 = bV2 - j.mul(invMassB) * t
            
            // new angular velocity
            aW2 = aW2 + aRf.mul(j).mul(fixDouble: invInerA)
            bW2 = bW2 - bRf.mul(j).mul(fixDouble: invInerB)
        }
        
        return DmSolution(
            velA: Velocity(linear: aV2, angular: aW2),
            velB: Velocity(linear: bV2, angular: bW2),
            isImpact: true
        )
    }
}
