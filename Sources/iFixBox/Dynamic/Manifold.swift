//
//  Manifold.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

struct Other {
    let isA: Bool
    let index: Int
}

struct ImpactSolution {
    
    static let noImpact = ImpactSolution(velA: .zero, velB: .zero, isImpact: false)

    let velA: Velocity
    let velB: Velocity
    let isImpact: Bool
}


struct LetBody {

    let startVel: Velocity
    let invMass: FixFloat
    let invInertia: FixDouble
    
    @inlinable
    var isDynamic: Bool { invMass != 0 }
    
    @inlinable
    init(body: Body) {
        self.startVel = body.velocity
        self.invMass = body.invMass
        self.invInertia = body.invInertia
    }
}

struct Manifold {
    
    let a: LetBody
    let b: LetBody
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
        self.a = LetBody(body: a)
        self.b = LetBody(body: b)
        self.iA = iA
        self.iB = iB
        
        // Normal is always look at A * <-| * B
        self.n = contact.normal
        
        // -(1 + e)
        ke = -max(a.material.bounce, b.material.bounce) - .unit
        q = (a.material.friction + b.material.friction) >> 1
        
        if contact.penetration < 0 {
            let l = -contact.penetration
            let maxBias = l.mul(iTimeStep)
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
    
    mutating func set(vA: Int, vB: Int) {
        self.vA = vA
        self.vB = vB
    }
    
    func resolve(varA: VarBody, varB: VarBody) -> ImpactSolution {
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
        var aV2 = aV1 + i.mul(a.invMass) * n
        var bV2 = bV1 - i.mul(b.invMass) * n
        
        // new angular velocity
        var aW2 = aW1 + aRn.mul(i).mul(fixDouble: a.invInertia)
        var bW2 = bW1 - bRn.mul(i).mul(fixDouble: b.invInertia)

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
            aV2 = aV2 + j.mul(a.invMass) * t
            bV2 = bV2 - j.mul(b.invMass) * t
        
            // new angular velocity
            aW2 = aW2 + aRf.mul(j).mul(fixDouble: a.invInertia)
            bW2 = bW2 - bRf.mul(j).mul(fixDouble: b.invInertia)
        }

        return ImpactSolution(
            velA: Velocity(linear: aV2, angular: aW2),
            velB: Velocity(linear: bV2, angular: bW2),
            isImpact: true
        )
    }

    

    func resolve(varA: VarBody) -> ImpactSolution {
        // start linear and angular velocity for A and B
        let aV1 = varA.velocity.linear
        let aW1 = varA.velocity.angular

        let bV1 = b.startVel.linear
        let bW1 = b.startVel.angular
        
        // relative velocity
        let rV1 = aV1 - bV1 + aR.crossProduct(aW1) - bR.crossProduct(bW1)

        var rV1proj = rV1.dotProduct(n)
        
        // only if getting closer
        guard rV1proj < bias else {
            return .noImpact
        }
        
        rV1proj = min(rV1proj, -bias)
        
        // normal impulse
        // -(1 + e) * rV1 * n / (1 / Ma + (aR * t)^2 / aI)
        
        let iNum = rV1proj.mul(ke)
        let i = iNum.div(iDen)

        // new linear velocity
        var aV2 = aV1 + i.mul(a.invMass) * n
        
        // new angular velocity
        var aW2 = aW1 + aRn.mul(i).mul(fixDouble: a.invInertia)

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
            aV2 = aV2 + j.mul(a.invMass) * t
        
            // new angular velocity
            aW2 = aW2 + aRf.mul(j).mul(fixDouble: a.invInertia)
        }

        return ImpactSolution(velA: Velocity(linear: aV2, angular: aW2), velB: .zero, isImpact: true)
    }
  
    func other(index: Int) -> Other {
        if index == vA {
            return Other(isA: true, index: vB)
        } else {
            return Other(isA: false, index: vA)
        }
    }

    func possibleVelocity(varA: Velocity, varB: Velocity) -> FixFloat {

        let aV1 = varA.linear
        let aW1 = varA.angular


        let av = aV1 + aR.crossProduct(aW1)
        let an = av.dotProduct(n)
        
//        if an >= 0 {
//            return .zero
//        }

        let bV1 = varB.linear
        let bW1 = varB.angular

        let bv = bV1 + bR.crossProduct(bW1)
        
        let bn = bv.dotProduct(n)
        
        let rn = an - bn
        
        if rn >= -20 {
            return .unit
        }

        if an <= bn {
            return 0
        }

        if an == .zero {
            return 0
        }

        return bn.div(an)
    }
    
    func possibleVelocity(varA: Velocity) -> FixFloat {
        self.possibleVelocity(varA: varA, varB: b.startVel)
    }
    
}
